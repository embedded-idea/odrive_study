#!/usr/bin/env python3
"""
ODrive 2804 + AS5047P 本地 Web GUI（调试版）

用法:
    ~/.odrive-venv/bin/python tools/odrv_gui.py
    浏览器打开 http://localhost:8765

特点:
    - 状态卡片 + 速度/位置/电流 三路实时曲线
    - 速度滑块、快捷档位、点动、急停
    - 常用调试参数在线读写
    - 错误码翻译成名字
    - 导出 CSV
    - 只用 Python 标准库 + odrive 包
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import odrive

try:
    from OdrvTool.device import (
        AXIS_ERR, AXIS_STATE, CTRL_ERR, ENC_ERR, MOTOR_ERR, ODRV_ERR,
        decode_error,
    )
except Exception:  # 独立运行时的最小回退
    ODRV_ERR = {0x02: "DC_BUS_UNDER_VOLTAGE", 0x04: "DC_BUS_OVER_VOLTAGE",
                0x08: "DC_BUS_OVER_REGEN_CURRENT", 0x10: "DC_BUS_OVER_CURRENT"}
    AXIS_ERR = {0x01: "INVALID_STATE", 0x40: "MOTOR_FAILED",
                0x100: "ENCODER_FAILED", 0x200: "CONTROLLER_FAILED"}
    MOTOR_ERR = {0x08: "DRV_FAULT", 0x1000: "CURRENT_LIMIT_VIOLATION",
                 0x1000000: "SYSTEM_LEVEL"}
    ENC_ERR = {0x80: "ABS_SPI_COM_FAIL", 0x02: "CPR_POLEPAIRS_MISMATCH"}
    CTRL_ERR = {0x01: "OVERSPEED"}
    AXIS_STATE = {1: "IDLE", 4: "MOTOR_CALIBRATION", 7: "ENCODER_OFFSET_CALIBRATION",
                  8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN"}

    def decode_error(mask, table):
        if not mask:
            return "OK"
        names = [n for b, n in table.items() if mask & b]
        return "0x%x (%s)" % (mask, " | ".join(names)) if names else "0x%x" % mask


LOCK = threading.RLock()
DEV = None
LAST_ERROR = ""

PARAMS = {
    "vel_gain": "axis0.controller.config.vel_gain",
    "vel_integrator_gain": "axis0.controller.config.vel_integrator_gain",
    "pos_gain": "axis0.controller.config.pos_gain",
    "vel_limit": "axis0.controller.config.vel_limit",
    "vel_ramp_rate": "axis0.controller.config.vel_ramp_rate",
    "current_lim": "axis0.motor.config.current_lim",
    "torque_constant": "axis0.motor.config.torque_constant",
    "enc_bandwidth": "axis0.encoder.config.bandwidth",
}


def rd(path):
    o = DEV
    for a in path.split("."):
        o = getattr(o, a)
    return o


def wr(path, value):
    o = DEV
    parts = path.split(".")
    for a in parts[:-1]:
        o = getattr(o, a)
    setattr(o, parts[-1], value)


def connect():
    global DEV, LAST_ERROR
    with LOCK:
        try:
            DEV = odrive.find_sync(timeout=6)
            LAST_ERROR = ""
            return True
        except AssertionError:
            DEV = None
            LAST_ERROR = "设备被其它程序占用（关掉 odrivetool / 其它脚本）"
            return False
        except Exception as e:
            DEV = None
            LAST_ERROR = "%s: %s" % (type(e).__name__, e)
            return False


def get_state():
    global DEV, LAST_ERROR
    with LOCK:
        if DEV is None:
            return {"connected": False, "error": LAST_ERROR}
        out = {"connected": True, "error": ""}
        try:
            out["vbus"] = rd("vbus_voltage")
            out["ibus"] = rd("ibus")
            out["state"] = rd("axis0.current_state")
            out["state_name"] = AXIS_STATE.get(out["state"], str(out["state"]))
            out["vel"] = rd("axis0.encoder.vel_estimate")
            out["vel_set"] = rd("axis0.controller.vel_setpoint")
            out["pos"] = rd("axis0.encoder.pos_estimate")
            out["pos_abs"] = rd("axis0.encoder.pos_abs")
            out["iq"] = rd("axis0.motor.current_control.Iq_measured")
            out["ia"] = rd("axis0.motor.current_meas_phA")
            out["ib"] = rd("axis0.motor.current_meas_phB")
            out["ic"] = rd("axis0.motor.current_meas_phC")
            out["sensorless"] = rd("axis0.config.enable_sensorless_mode")
            out["pre_cal"] = rd("axis0.encoder.config.pre_calibrated")
            out["uptime"] = rd("system_stats.uptime")
            out["fw"] = "v%d.%d.%d" % (
                rd("fw_version_major"), rd("fw_version_minor"), rd("fw_version_revision"))

            errs = {
                "odrv": rd("error"),
                "axis": rd("axis0.error"),
                "motor": rd("axis0.motor.error"),
                "encoder": rd("axis0.encoder.error"),
                "controller": rd("axis0.controller.error"),
            }
            tables = {"odrv": ODRV_ERR, "axis": AXIS_ERR, "motor": MOTOR_ERR,
                      "encoder": ENC_ERR, "controller": CTRL_ERR}
            out["errs"] = {k: int(v) for k, v in errs.items()}
            out["err_text"] = {k: decode_error(v, tables[k]) for k, v in errs.items()}
            out["error_ok"] = not any(errs.values())
        except Exception as e:
            LAST_ERROR = "%s: %s" % (type(e).__name__, e)
            DEV = None
            return {"connected": False, "error": LAST_ERROR}
        return out


def capture(n=300):
    """高速采样三相电流（USB 轮询，非 8kHz 连续采样）"""
    global DEV, LAST_ERROR
    with LOCK:
        if DEV is None:
            return {"ok": False, "msg": "未连接: " + LAST_ERROR}
        try:
            n = max(1, min(int(n), 2000))
            t0 = time.perf_counter()
            ts, a, b, c = [], [], [], []
            for _ in range(n):
                ts.append((time.perf_counter() - t0) * 1000.0)
                bi = rd("axis0.motor.current_meas_phB")
                ci = rd("axis0.motor.current_meas_phC")
                b.append(bi)
                c.append(ci)
                a.append(-(bi + ci))
        except Exception as e:
            LAST_ERROR = "%s: %s" % (type(e).__name__, e)
            DEV = None
            return {"ok": False, "msg": LAST_ERROR}
        dur = ts[-1] if ts else 0.0
        return {"ok": True, "t": ts, "a": a, "b": b, "c": c,
                "duration_ms": dur,
                "rate": (len(ts) / dur * 1000.0) if dur > 0 else 0.0}


def get_config():
    with LOCK:
        if DEV is None:
            return {"ok": False, "msg": "未连接"}
        out = {}
        for name, path in PARAMS.items():
            try:
                out[name] = rd(path)
            except Exception:
                out[name] = None
        return {"ok": True, "params": out}


def _ensure_closed_loop(mode):
    if DEV.axis0.current_state != 8:
        try:
            DEV.axis0.controller.input_vel = 0.0
        except Exception:
            pass
        DEV.axis0.controller.config.control_mode = mode
        DEV.axis0.controller.config.input_mode = 1
        DEV.axis0.requested_state = 8
        time.sleep(0.05)


def run_cmd(body):
    global DEV
    action = body.get("action")
    value = body.get("value")

    with LOCK:
        if DEV is None and not connect():
            return {"ok": False, "msg": "未连接: " + LAST_ERROR}
        try:
            if action == "start":
                DEV.axis0.controller.config.control_mode = 2
                DEV.axis0.controller.config.input_mode = 1
                try:
                    DEV.axis0.controller.input_vel = 0.0
                except Exception:
                    pass
                DEV.axis0.requested_state = 8
                return {"ok": True, "msg": "已进入闭环 (velocity, 0 turn/s)"}

            if action in ("stop", "estop"):
                try:
                    DEV.axis0.controller.input_vel = 0.0
                except Exception:
                    pass
                DEV.axis0.requested_state = 1
                return {"ok": True, "msg": "已停止 (IDLE)"}

            if action == "clear":
                DEV.clear_errors()
                return {"ok": True, "msg": "错误已清除"}

            if action == "vel":
                v = float(value)
                _ensure_closed_loop(2)
                DEV.axis0.controller.config.input_mode = 1
                DEV.axis0.controller.input_vel = v
                return {"ok": True, "msg": "速度 %+.2f turn/s" % v}

            if action == "ramp":
                v = float(value)
                _ensure_closed_loop(2)
                DEV.axis0.controller.config.input_mode = 2
                DEV.axis0.controller.config.vel_ramp_rate = 1.0
                DEV.axis0.controller.input_vel = v
                return {"ok": True, "msg": "平滑速度 %+.2f turn/s" % v}

            if action == "pos":
                p = float(value)
                _ensure_closed_loop(3)
                DEV.axis0.controller.config.input_mode = 1
                DEV.axis0.controller.input_pos = p
                return {"ok": True, "msg": "目标位置 %.3f turn" % p}

            if action == "rel":
                p = float(value)
                _ensure_closed_loop(3)
                DEV.axis0.controller.config.input_mode = 1
                DEV.axis0.controller.move_incremental(p, True)
                return {"ok": True, "msg": "相对移动 %+.3f turn" % p}

            if action == "cal_motor":
                DEV.axis0.requested_state = 4
                return {"ok": True, "msg": "电机标定开始"}

            if action == "cal_enc":
                if DEV.axis0.current_state != 1:
                    DEV.axis0.requested_state = 1
                    time.sleep(0.3)
                DEV.axis0.requested_state = 7
                return {"ok": True, "msg": "编码器偏置标定开始（电机转动约 1.1 圈）"}

            if action == "set_many":
                params = body.get("params") or {}
                applied = []
                for name, v in params.items():
                    if name in PARAMS:
                        wr(PARAMS[name], float(v))
                        applied.append(name)
                return {"ok": True, "msg": "已应用: " + ", ".join(applied)}

            if action == "save":
                try:
                    DEV.save_configuration()
                except Exception:
                    pass
                DEV = None
                return {"ok": True, "msg": "已保存，设备重启中… 稍后点“重连”"}

            if action == "reconnect":
                ok = connect()
                return {"ok": ok, "msg": "已重连" if ok else "连接失败: " + LAST_ERROR}

            return {"ok": False, "msg": "未知命令: %r" % (action,)}
        except Exception as e:
            return {"ok": False, "msg": "%s: %s" % (type(e).__name__, e)}


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ODrive 2804 Console</title>
<style>
  :root{
    --bg:#0e1116; --panel:#161b23; --line:#252c38; --text:#e6edf3;
    --muted:#8b98a9; --accent:#4da3ff; --ok:#35c46a; --warn:#f0b429; --bad:#ff5c5c;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Noto Sans SC",sans-serif}
  header{display:flex;align-items:center;gap:12px;padding:10px 18px;border-bottom:1px solid var(--line);background:#11161d;position:sticky;top:0;z-index:10}
  header h1{font-size:15px;margin:0;font-weight:600}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--bad)}
  .dot.on{background:var(--ok)}
  #conn{color:var(--muted);font-size:12px}
  #headright{margin-left:auto;display:flex;align-items:center;gap:10px}
  #estop{background:#8e1b1b;border:1px solid #d33;color:#fff;font-weight:700;padding:8px 18px;border-radius:8px;font-size:14px}
  #estop:hover{background:#a52222}
  main{max-width:1180px;margin:0 auto;padding:16px 18px 40px}
  .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin-bottom:12px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
  .card .k{color:var(--muted);font-size:12px}
  .card .v{font-size:19px;font-weight:600;margin-top:2px;font-variant-numeric:tabular-nums}
  .card .s{font-size:11px;color:var(--muted);margin-top:1px}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:12px}
  .row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:7px 0}
  label{color:var(--muted);font-size:12px;min-width:72px}
  input[type=number],select{width:110px;background:#0d1218;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:7px 9px;font-size:13px}
  input[type=range]{width:260px;accent-color:var(--accent)}
  button{background:#1d2633;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:7px 12px;font-size:13px;cursor:pointer}
  button:hover{border-color:#3a4a61}
  button.primary{background:#173a5e;border-color:#2b5f92}
  button.ok{background:#12341f;border-color:#1f6b3a}
  button.danger{background:#3a1414;border-color:#7a2626}
  button.small{padding:5px 9px;font-size:12px}
  .charts{display:grid;grid-template-columns:1fr;gap:10px;margin-bottom:12px}
  .chart{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:6px 8px}
  .chart .t{font-size:12px;color:var(--muted);margin:2px 0 0 4px}
  canvas{width:100%;height:130px;display:block}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
  .errline{font:12px/1.7 ui-monospace,Menlo,Consolas,monospace;color:var(--muted)}
  .errline .bad{color:var(--bad)}
  .errline .ok{color:var(--ok)}
  .pgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:8px}
  .pitem{display:flex;align-items:center;gap:8px}
  .pitem label{min-width:110px}
  .pitem input{width:100px}
  .log{background:#0b0f14;border:1px solid var(--line);border-radius:10px;padding:8px 10px;height:96px;overflow:auto;font:12px/1.6 ui-monospace,Menlo,Consolas,monospace;color:var(--muted)}
  .log .ok{color:var(--ok)} .log .bad{color:var(--bad)}
  .hint{color:var(--muted);font-size:11px}
  @media(max-width:820px){.grid2{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <span class="dot" id="dot"></span>
  <h1>ODrive 2804 Console</h1>
  <span id="conn">连接中…</span>
  <div id="headright">
    <span class="hint">Space / Esc = 急停</span>
    <button id="estop">■ 急停</button>
  </div>
</header>
<main>
  <div class="cards">
    <div class="card"><div class="k">母线</div><div class="v" id="vbus">–</div><div class="s" id="ibus">–</div></div>
    <div class="card"><div class="k">状态</div><div class="v" id="state" style="font-size:15px">–</div><div class="s" id="mode">–</div></div>
    <div class="card"><div class="k">速度</div><div class="v" id="vel">–</div><div class="s" id="velset">–</div></div>
    <div class="card"><div class="k">位置</div><div class="v" id="pos">–</div><div class="s" id="posabs">–</div></div>
    <div class="card"><div class="k">Iq</div><div class="v" id="iq">–</div><div class="s">q 轴电流</div></div>
    <div class="card"><div class="k">Ia / Ib / Ic</div><div class="v" id="iabc" style="font-size:13px">–</div><div class="s">三相电流 (A)</div></div>
    <div class="card"><div class="k">错误</div><div class="v" id="err">–</div><div class="s" id="errsrc">–</div></div>
  </div>

  <div class="panel">
    <div class="row">
      <button class="ok primary" id="btnStart">▶ 启动闭环</button>
      <button class="danger" id="btnStop">■ 停止</button>
      <button id="btnClear">清除错误</button>
      <button id="btnReconnect">重连</button>
      <span class="hint" id="fw">–</span>
    </div>
    <div class="row">
      <label>速度</label>
      <input type="range" id="velRange" min="-5" max="5" step="0.05" value="0">
      <input type="number" id="velIn" value="0.2" step="0.05">
      <button class="primary" id="btnVel">应用</button>
      <button id="btnRamp">平滑</button>
      <span class="hint">turn/s（1 = 60 RPM）</span>
    </div>
    <div class="row">
      <label>快捷</label>
      <button class="small" data-vel="-2">-2</button>
      <button class="small" data-vel="-1">-1</button>
      <button class="small" data-vel="-0.5">-0.5</button>
      <button class="small" data-vel="-0.1">-0.1</button>
      <button class="small" data-vel="0">0</button>
      <button class="small" data-vel="0.1">0.1</button>
      <button class="small" data-vel="0.5">0.5</button>
      <button class="small" data-vel="1">1</button>
      <button class="small" data-vel="2">2</button>
    </div>
    <div class="row">
      <label>位置</label>
      <input type="number" id="posIn" value="1.0" step="0.1">
      <button class="primary" id="btnPos">绝对</button>
      <button id="btnRel">相对</button>
      <button class="small" data-jog="-1">-1</button>
      <button class="small" data-jog="-0.1">-0.1</button>
      <button class="small" data-jog="0.1">+0.1</button>
      <button class="small" data-jog="1">+1</button>
    </div>
  </div>

  <div class="charts">
    <div class="chart"><div class="t">速度 (turn/s) — 灰虚线=目标，蓝=实测</div><canvas id="chartVel"></canvas></div>
    <div class="chart"><div class="t">位置 (turn)</div><canvas id="chartPos"></canvas></div>
    <div class="chart"><div class="t">Iq (A)</div><canvas id="chartIq"></canvas></div>
  </div>

  <div class="panel">
    <div class="row">
      <b style="font-size:13px">三相电流</b>
      <label>采样点数</label>
      <input type="number" id="capN" value="300" step="50" min="20" max="2000">
      <button class="primary small" id="btnCapture">采集</button>
      <label style="min-width:auto"><input type="checkbox" id="capAuto"> 自动（1.5s）</label>
      <span class="hint" id="capStats">Ia = -(Ib + Ic)（v3.6 只测 B/C；USB 读取非严格同步）</span>
    </div>
    <div class="chart"><canvas id="chartIabc"></canvas></div>
  </div>

  <div class="grid2">
    <div class="panel">
      <div class="row"><b style="font-size:13px">调试参数</b>
        <button class="small" id="btnReadCfg">读取</button>
        <button class="small primary" id="btnApplyCfg">应用</button>
        <button class="small" id="btnSave">保存并重启</button>
      </div>
      <div class="pgrid">
        <div class="pitem"><label>vel_gain</label><input type="number" id="p_vel_gain" step="0.01"></div>
        <div class="pitem"><label>vel_int_gain</label><input type="number" id="p_vel_integrator_gain" step="0.01"></div>
        <div class="pitem"><label>pos_gain</label><input type="number" id="p_pos_gain" step="0.5"></div>
        <div class="pitem"><label>vel_limit</label><input type="number" id="p_vel_limit" step="0.5"></div>
        <div class="pitem"><label>vel_ramp_rate</label><input type="number" id="p_vel_ramp_rate" step="0.5"></div>
        <div class="pitem"><label>current_lim</label><input type="number" id="p_current_lim" step="0.1"></div>
        <div class="pitem"><label>torque_constant</label><input type="number" id="p_torque_constant" step="0.005"></div>
        <div class="pitem"><label>enc_bandwidth</label><input type="number" id="p_enc_bandwidth" step="50"></div>
      </div>
      <div class="row">
        <button class="small" id="btnCalMotor">电机标定</button>
        <button class="small danger" id="btnCalEnc">编码器偏置标定</button>
        <button class="small" id="btnCsv">导出 CSV</button>
      </div>
    </div>
    <div class="panel">
      <div class="row"><b style="font-size:13px">错误详情</b><span class="hint" id="uptime"></span></div>
      <div class="errline" id="errbox">–</div>
    </div>
  </div>

  <div class="log" id="log"></div>
</main>

<script>
const $ = id => document.getElementById(id);
const NAMES = {0:"UNDEFINED",1:"IDLE",2:"STARTUP",3:"FULL_CAL",4:"MOTOR_CAL",
               6:"INDEX_SEARCH",7:"ENC_OFFSET",8:"CLOSED_LOOP",9:"LOCKIN",11:"HOMING"};
const MAXHIST = 300;
let hist = [];

function log(msg, ok){
  const el = document.createElement('div');
  el.className = ok ? 'ok' : 'bad';
  el.textContent = new Date().toLocaleTimeString() + '  ' + msg;
  $('log').prepend(el);
  while ($('log').children.length > 60) $('log').lastChild.remove();
}

async function api(path, body){
  const opt = body === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)
  };
  const r = await fetch(path, opt);
  return await r.json();
}

function send(action, value, extra){
  const body = Object.assign({action, value}, extra || {});
  api('/api/cmd', body).then(r => log(r.msg, r.ok)).catch(e => log(e.message, false));
}

function setConn(on, text){
  $('dot').className = 'dot' + (on ? ' on' : '');
  $('conn').textContent = on ? '已连接' : ('未连接 ' + (text || ''));
}

function fmt(v, d, u){ return (v === null || v === undefined) ? '–' : v.toFixed(d) + (u || ''); }

async function refresh(){
  try{
    const s = await api('/api/state');
    if (!s.connected){ setConn(false, s.error); return; }
    setConn(true);
    $('fw').textContent = s.fw + '  uptime ' + Math.round(s.uptime) + 's';
    $('vbus').textContent = fmt(s.vbus, 2, ' V');
    $('ibus').textContent = fmt(s.ibus, 2, ' A');
    $('state').textContent = NAMES[s.state] || s.state;
    $('mode').textContent = s.sensorless ? 'sensorless' : 'encoder';
    $('vel').textContent = fmt(s.vel, 3, ' t/s');
    $('velset').textContent = 'set ' + fmt(s.vel_set, 3, ' t/s');
    $('pos').textContent = fmt(s.pos, 3, ' turn');
    $('posabs').textContent = 'abs ' + (s.pos_abs === null ? '–' : s.pos_abs);
    $('iq').textContent = fmt(s.iq, 3, ' A');
    $('iabc').textContent = fmt(s.ia, 2) + ' / ' + fmt(s.ib, 2) + ' / ' + fmt(s.ic, 2);
    $('err').textContent = s.error_ok ? 'OK' : 'FAULT';
    $('err').style.color = s.error_ok ? 'var(--ok)' : 'var(--bad)';
    $('errsrc').textContent = s.error_ok ? 'no error' : '见右侧详情';
    $('uptime').textContent = 'uptime ' + Math.round(s.uptime) + ' s';
    const lines = [];
    for (const k of ['odrv','axis','motor','encoder','controller']) {
      const t = s.err_text[k];
      lines.push(k + ': ' + (t === 'OK'
        ? '<span class="ok">OK</span>'
        : '<span class="bad">' + t + '</span>'));
    }
    $('errbox').innerHTML = lines.join('<br>');
    hist.push({t: Date.now()/1000, vel: s.vel||0, set: s.vel_set||0,
               pos: s.pos||0, iq: s.iq||0, vbus: s.vbus||0});
    if (hist.length > MAXHIST) hist.shift();
    drawAll();
  }catch(err){ setConn(false, err.message); }
}

function drawChart(canvas, series){
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w*dpr) || canvas.height !== Math.round(h*dpr)){
    canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);
  const pad = 6;
  let m = 0.05;
  for (const s of series) for (const p of hist) m = Math.max(m, Math.abs(p[s.key]||0));
  m *= 1.2;
  const y = v => h/2 - (v/m) * (h/2 - pad);
  const x = i => pad + (hist.length <= 1 ? 0 : i/(hist.length-1) * (w - 2*pad));
  ctx.strokeStyle = '#252c38'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0,y(0)); ctx.lineTo(w,y(0)); ctx.stroke();
  for (const s of series){
    ctx.strokeStyle = s.color; ctx.lineWidth = 2;
    ctx.setLineDash(s.dash ? [5,4] : []);
    ctx.beginPath();
    hist.forEach((p,i) => { const yy = y(p[s.key]||0); i ? ctx.lineTo(x(i),yy) : ctx.moveTo(x(i),yy); });
    ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.fillStyle = '#8b98a9'; ctx.font = '11px sans-serif';
  ctx.fillText('+' + m.toFixed(2), 6, 12);
  ctx.fillText('-' + m.toFixed(2), 6, h - 4);
}

function drawAll(){
  drawChart($('chartVel'), [{key:'set', color:'#8b98a9', dash:true},
                            {key:'vel', color:'#4da3ff'}]);
  drawChart($('chartPos'), [{key:'pos', color:'#35c46a'}]);
  drawChart($('chartIq'),  [{key:'iq',  color:'#f0b429'}]);
}

function drawMulti(canvas, series){
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w*dpr) || canvas.height !== Math.round(h*dpr)){
    canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);
  const pad = 6;
  let m = 0.05, n = 0;
  for (const s of series){ n = Math.max(n, s.data.length);
    for (const v of s.data) m = Math.max(m, Math.abs(v)); }
  m *= 1.2;
  const len = Math.max(1, n);
  const y = v => h/2 - (v/m) * (h/2 - pad);
  const x = i => pad + i/(len-1) * (w - 2*pad);
  ctx.strokeStyle = '#252c38'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0,y(0)); ctx.lineTo(w,y(0)); ctx.stroke();
  for (const s of series){
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.6;
    ctx.beginPath();
    s.data.forEach((v,i) => { i ? ctx.lineTo(x(i),y(v)) : ctx.moveTo(x(i),y(v)); });
    ctx.stroke();
  }
  ctx.fillStyle = '#8b98a9'; ctx.font = '11px sans-serif';
  ctx.fillText('+' + m.toFixed(2) + ' A', 6, 12);
  ctx.fillText('-' + m.toFixed(2) + ' A', 6, h - 4);
  ctx.fillStyle = '#4da3ff'; ctx.fillText('Ia', w - 74, 12);
  ctx.fillStyle = '#35c46a'; ctx.fillText('Ib', w - 52, 12);
  ctx.fillStyle = '#f0b429'; ctx.fillText('Ic', w - 30, 12);
}

let lastCap = null;
async function captureNow(){
  const n = parseInt($('capN').value) || 300;
  try{
    const r = await fetch('/api/capture?n=' + n).then(x => x.json());
    if (!r.ok){ log(r.msg, false); return; }
    lastCap = [r.a, r.b, r.c];
    drawMulti($('chartIabc'), [
      {data:r.a, color:'#4da3ff'},
      {data:r.b, color:'#35c46a'},
      {data:r.c, color:'#f0b429'},
    ]);
    const rms = arr => Math.sqrt(arr.reduce((s,v)=>s+v*v,0)/arr.length);
    const pk = arr => Math.max(...arr.map(Math.abs));
    $('capStats').textContent = n + ' 点 / ' + r.duration_ms.toFixed(0) + ' ms / ' +
      r.rate.toFixed(0) + ' Hz   RMS: A ' + rms(r.a).toFixed(3) +
      '  B ' + rms(r.b).toFixed(3) + '  C ' + rms(r.c).toFixed(3) +
      ' A   peak ' + Math.max(pk(r.a), pk(r.b), pk(r.c)).toFixed(3) + ' A';
  }catch(e){ log(e.message, false); }
}

let capTimer = null;
function redrawMulti(){
  if (!lastCap) return;
  drawMulti($('chartIabc'), [
    {data:lastCap[0], color:'#4da3ff'},
    {data:lastCap[1], color:'#35c46a'},
    {data:lastCap[2], color:'#f0b429'},
  ]);
}

async function readCfg(){
  const r = await api('/api/config');
  if (!r.ok){ log(r.msg, false); return; }
  for (const [k,v] of Object.entries(r.params)){
    const el = $('p_' + k);
    if (el && v !== null) el.value = (+v).toFixed(4).replace(/0+$/,'').replace(/\.$/,'');
  }
  log('参数已读取', true);
}

function applyCfg(){
  const params = {};
  for (const k of ['vel_gain','vel_integrator_gain','pos_gain','vel_limit',
                   'vel_ramp_rate','current_lim','torque_constant','enc_bandwidth']){
    const el = $('p_' + k);
    if (el && el.value !== '') params[k] = parseFloat(el.value);
  }
  send('set_many', null, {params});
}

$('estop').onclick = () => send('estop');
$('btnStart').onclick = () => send('start');
$('btnStop').onclick = () => send('stop');
$('btnClear').onclick = () => send('clear');
$('btnReconnect').onclick = () => send('reconnect');
$('btnVel').onclick = () => send('vel', parseFloat($('velIn').value));
$('btnRamp').onclick = () => send('ramp', parseFloat($('velIn').value));
$('btnPos').onclick = () => send('pos', parseFloat($('posIn').value));
$('btnRel').onclick = () => send('rel', parseFloat($('posIn').value));
$('velRange').oninput = e => { $('velIn').value = e.target.value; };
$('velIn').oninput = e => { $('velRange').value = e.target.value; };
document.querySelectorAll('[data-vel]').forEach(b => b.onclick = () => {
  $('velIn').value = b.dataset.vel; $('velRange').value = b.dataset.vel;
  send('vel', parseFloat(b.dataset.vel));
});
document.querySelectorAll('[data-jog]').forEach(b => b.onclick = () => {
  send('rel', parseFloat(b.dataset.jog));
});
$('btnReadCfg').onclick = readCfg;
$('btnApplyCfg').onclick = applyCfg;
$('btnSave').onclick = () => { if (confirm('保存配置并重启？')) send('save'); };
$('btnCalMotor').onclick = () => { if (confirm('开始电机标定？')) send('cal_motor'); };
$('btnCalEnc').onclick = () => {
  if (confirm('编码器偏置标定：电机会转动约 1.1 圈，确认？')) send('cal_enc');
};
$('btnCsv').onclick = () => {
  let csv = 'time,vel,vel_set,pos,iq,vbus\n';
  for (const p of hist) csv += [p.t.toFixed(3),p.vel,p.set,p.pos,p.iq,p.vbus].join(',') + '\n';
  const blob = new Blob([csv], {type:'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'odrive_log.csv';
  a.click();
  URL.revokeObjectURL(a.href);
};
$('btnCapture').onclick = captureNow;
$('capAuto').onchange = e => {
  if (capTimer){ clearInterval(capTimer); capTimer = null; }
  if (e.target.checked){ captureNow(); capTimer = setInterval(captureNow, 1500); }
};
document.addEventListener('keydown', e => {
  if (e.target.matches('input,select,textarea')) return;
  if (e.code === 'Space' || e.key === 'Escape'){ e.preventDefault(); send('estop'); }
});

$('velRange').value = $('velIn').value;
refresh();
readCfg();
setInterval(refresh, 500);
window.addEventListener('resize', () => { drawAll(); redrawMulti(); });
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, HTML, "text/html; charset=utf-8")
        elif self.path.startswith("/api/state"):
            self._send(200, json.dumps(get_state()))
        elif self.path.startswith("/api/config"):
            self._send(200, json.dumps(get_config()))
        elif self.path.startswith("/api/capture"):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)
            try:
                n = int(q.get("n", ["300"])[0])
            except ValueError:
                n = 300
            self._send(200, json.dumps(capture(n)))
        else:
            self._send(404, "{}")

    def do_POST(self):
        if self.path.startswith("/api/cmd"):
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                body = {}
            self._send(200, json.dumps(run_cmd(body)))
        else:
            self._send(404, "{}")


def main():
    ap = argparse.ArgumentParser(description="ODrive 2804 local web GUI")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--no-connect", action="store_true")
    args = ap.parse_args()

    if not args.no_connect:
        print("connecting to ODrive ...")
        if connect():
            print("connected")
        else:
            print("connect failed:", LAST_ERROR)

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("GUI: http://localhost:%d" % args.port)
    print("Ctrl-C 退出（退出不会停机，需要先在页面点“停止”）")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
