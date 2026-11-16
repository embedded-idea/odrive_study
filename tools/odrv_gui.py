#!/usr/bin/env python3
"""
ODrive 2804 + AS5047P 本地 Web GUI

用法:
    ~/.odrive-venv/bin/python odrv_gui.py
    浏览器打开 http://localhost:8765

说明:
    - 只用 Python 标准库 + odrive 包，无需安装前端依赖
    - 同一时间只能有一个程序占用 ODrive（先关掉 odrivetool）
    - 所有操作都通过 USB 直接下发，和 odrivetool 的命令等价
"""

import argparse
import json
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import odrive

LOCK = threading.RLock()
DEV = None
LAST_ERROR = ""

PROPS = {
    "vbus": "vbus_voltage",
    "ibus": "ibus",
    "state": "axis0.current_state",
    "axis_err": "axis0.error",
    "motor_err": "axis0.motor.error",
    "enc_err": "axis0.encoder.error",
    "ctrl_err": "axis0.controller.error",
    "vel": "axis0.encoder.vel_estimate",
    "vel_set": "axis0.controller.vel_setpoint",
    "pos": "axis0.encoder.pos_estimate",
    "pos_abs": "axis0.encoder.pos_abs",
    "iq": "axis0.motor.current_control.Iq_measured",
    "pre_cal": "axis0.encoder.config.pre_calibrated",
    "sensorless": "axis0.config.enable_sensorless_mode",
}


def rd(path):
    o = DEV
    for a in path.split("."):
        o = getattr(o, a)
    return o


def connect():
    global DEV, LAST_ERROR
    with LOCK:
        try:
            DEV = odrive.find_sync(timeout=6)
            LAST_ERROR = ""
            return True
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
            for k, p in PROPS.items():
                if k == "vbus":
                    continue
                try:
                    out[k] = rd(p)
                except Exception:
                    out[k] = None
        except Exception as e:
            LAST_ERROR = "%s: %s" % (type(e).__name__, e)
            DEV = None
            return {"connected": False, "error": LAST_ERROR}
        return out


def _ensure_closed_loop(mode):
    if DEV.axis0.current_state != 8:
        try:
            # 先把速度指令清零，避免进闭环瞬间冲到上一次的 input_vel
            DEV.axis0.controller.input_vel = 0.0
        except Exception:
            pass
        DEV.axis0.controller.config.control_mode = mode
        DEV.axis0.controller.config.input_mode = 1
        DEV.axis0.requested_state = 8
        time.sleep(0.05)


def run_cmd(action, value):
    global DEV
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

            if action == "stop":
                try:
                    DEV.axis0.controller.input_vel = 0.0
                except Exception:
                    pass
                DEV.axis0.requested_state = 1
                return {"ok": True, "msg": "已停止 (IDLE)"}

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
                return {"ok": True, "msg": "目标位置 %.2f turn" % p}

            if action == "rel":
                p = float(value)
                _ensure_closed_loop(3)
                DEV.axis0.controller.config.input_mode = 1
                DEV.axis0.controller.move_incremental(p, True)
                return {"ok": True, "msg": "相对移动 %+.2f turn" % p}

            if action == "clear":
                DEV.clear_errors()
                return {"ok": True, "msg": "错误已清除"}

            if action == "cal_motor":
                DEV.axis0.requested_state = 4
                return {"ok": True, "msg": "电机标定开始 (云台为空操作)"}

            if action == "cal_enc":
                if DEV.axis0.current_state != 1:
                    DEV.axis0.requested_state = 1
                    time.sleep(0.3)
                DEV.axis0.requested_state = 7
                return {"ok": True, "msg": "编码器偏置标定开始，电机会转动约 1.1 圈"}

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
  body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,"Noto Sans SC",sans-serif}
  header{display:flex;align-items:center;gap:12px;padding:14px 20px;border-bottom:1px solid var(--line);background:#11161d;position:sticky;top:0;z-index:10}
  header h1{font-size:16px;margin:0;font-weight:600}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--bad)}
  .dot.on{background:var(--ok)}
  #conn{color:var(--muted);font-size:12px}
  main{max-width:1100px;margin:0 auto;padding:18px 20px 40px}
  .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:16px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
  .card .k{color:var(--muted);font-size:12px}
  .card .v{font-size:20px;font-weight:600;margin-top:4px;font-variant-numeric:tabular-nums}
  .card .s{font-size:12px;color:var(--muted);margin-top:2px}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:16px}
  .row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:8px 0}
  label{color:var(--muted);font-size:13px;min-width:96px}
  input[type=number]{width:120px;background:#0d1218;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:8px 10px;font-size:14px}
  button{background:#1d2633;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:8px 14px;font-size:13px;cursor:pointer}
  button:hover{border-color:#3a4a61}
  button.primary{background:#173a5e;border-color:#2b5f92}
  button.ok{background:#12341f;border-color:#1f6b3a}
  button.danger{background:#3a1414;border-color:#7a2626}
  .plot{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:8px;margin-bottom:16px}
  canvas{width:100%;height:220px;display:block}
  .log{background:#0b0f14;border:1px solid var(--line);border-radius:10px;padding:10px 12px;height:110px;overflow:auto;font:12px/1.6 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;color:var(--muted)}
  .log .ok{color:var(--ok)} .log .bad{color:var(--bad)}
  .hint{color:var(--muted);font-size:12px}
</style>
</head>
<body>
<header>
  <span class="dot" id="dot"></span>
  <h1>ODrive 2804 Console</h1>
  <span id="conn">连接中…</span>
</header>
<main>
  <div class="cards">
    <div class="card"><div class="k">母线电压</div><div class="v" id="vbus">–</div><div class="s" id="ibus">–</div></div>
    <div class="card"><div class="k">状态</div><div class="v" id="state">–</div><div class="s" id="mode">–</div></div>
    <div class="card"><div class="k">速度</div><div class="v" id="vel">–</div><div class="s" id="velset">–</div></div>
    <div class="card"><div class="k">位置</div><div class="v" id="pos">–</div><div class="s" id="posabs">–</div></div>
    <div class="card"><div class="k">Iq</div><div class="v" id="iq">–</div><div class="s">q 轴电流</div></div>
    <div class="card"><div class="k">错误</div><div class="v" id="err">–</div><div class="s" id="errsrc">–</div></div>
  </div>

  <div class="panel">
    <div class="row">
      <button class="ok primary" id="btnStart">▶ 启动闭环</button>
      <button class="danger" id="btnStop">■ 停止</button>
      <button id="btnClear">清除错误</button>
      <button id="btnReconnect">重连</button>
    </div>
    <div class="row">
      <label>速度 turn/s</label>
      <input type="number" id="velIn" value="0.2" step="0.05">
      <button class="primary" id="btnVel">应用</button>
      <button id="btnRamp">平滑加速</button>
      <span class="hint">1 turn/s = 60 RPM</span>
    </div>
    <div class="row">
      <label>位置 turn</label>
      <input type="number" id="posIn" value="1.0" step="0.1">
      <button class="primary" id="btnPos">绝对移动</button>
      <button id="btnRel">相对移动</button>
    </div>
    <div class="row">
      <label>标定</label>
      <button id="btnCalMotor">电机标定</button>
      <button class="danger" id="btnCalEnc">编码器偏置标定</button>
      <button id="btnSave">保存配置</button>
    </div>
  </div>

  <div class="plot"><canvas id="chart"></canvas></div>
  <div class="log" id="log"></div>
</main>

<script>
const $ = id => document.getElementById(id);
const NAMES = {0:"UNDEFINED",1:"IDLE",2:"STARTUP",3:"FULL_CAL",4:"MOTOR_CAL",
               6:"INDEX_SEARCH",7:"ENC_OFFSET",8:"CLOSED_LOOP",9:"LOCKIN",11:"HOMING"};
let hist = [];

function log(msg, ok){
  const el = document.createElement('div');
  el.className = ok ? 'ok' : 'bad';
  el.textContent = new Date().toLocaleTimeString() + '  ' + msg;
  $('log').prepend(el);
  while ($('log').children.length > 80) $('log').lastChild.remove();
}

async function api(path, body){
  const opt = body === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)
  };
  const r = await fetch(path, opt);
  return await r.json();
}

function send(action, value){
  api('/api/cmd', {action, value}).then(r => log(r.msg, r.ok)).catch(e => log(e.message, false));
}

function setConn(on, text){
  $('dot').className = 'dot' + (on ? ' on' : '');
  $('conn').textContent = on ? '已连接' : ('未连接 ' + (text || ''));
}

function fmt(v, digits, unit){
  return (v === null || v === undefined) ? '–' : v.toFixed(digits) + (unit || '');
}

async function refresh(){
  try{
    const s = await api('/api/state');
    if (!s.connected){ setConn(false, s.error); return; }
    setConn(true);
    $('vbus').textContent = fmt(s.vbus, 2, ' V');
    $('ibus').textContent = fmt(s.ibus, 2, ' A');
    $('state').textContent = NAMES[s.state] || s.state;
    $('mode').textContent = s.sensorless ? 'sensorless' : 'encoder';
    $('vel').textContent = fmt(s.vel, 3, ' t/s');
    $('velset').textContent = 'set ' + fmt(s.vel_set, 3, ' t/s');
    $('pos').textContent = fmt(s.pos, 3, ' turn');
    $('posabs').textContent = 'abs ' + (s.pos_abs === null ? '–' : s.pos_abs);
    $('iq').textContent = fmt(s.iq, 3, ' A');
    const e = [s.axis_err, s.motor_err, s.enc_err, s.ctrl_err];
    const bad = e.some(x => x);
    $('err').textContent = bad ? 'FAULT' : 'OK';
    $('err').style.color = bad ? 'var(--bad)' : 'var(--ok)';
    $('errsrc').textContent = bad
      ? 'a:0x' + (s.axis_err|0).toString(16) + ' m:0x' + (s.motor_err|0).toString(16) +
        ' e:0x' + (s.enc_err|0).toString(16) + ' c:0x' + (s.ctrl_err|0).toString(16)
      : 'no error';
    hist.push({t: Date.now()/1000, vel: s.vel || 0, set: s.vel_set || 0});
    if (hist.length > 240) hist.shift();
    draw();
  }catch(err){ setConn(false, err.message); }
}

const cv = $('chart');
const ctx = cv.getContext('2d');
function draw(){
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w*dpr) || cv.height !== Math.round(h*dpr)){
    cv.width = Math.round(w*dpr); cv.height = Math.round(h*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);
  const pad = 8;
  let m = 0.2;
  for (const p of hist) m = Math.max(m, Math.abs(p.vel), Math.abs(p.set));
  m *= 1.2;
  const y = v => h/2 - (v/m) * (h/2 - pad);
  const x = i => pad + (hist.length <= 1 ? 0 : i/(hist.length-1) * (w - 2*pad));

  ctx.strokeStyle = '#252c38'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0,y(0)); ctx.lineTo(w,y(0)); ctx.stroke();
  ctx.setLineDash([4,4]); ctx.strokeStyle = '#2b3a4d';
  ctx.beginPath(); ctx.moveTo(0,y(m/1.2)); ctx.lineTo(w,y(m/1.2)); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,y(-m/1.2)); ctx.lineTo(w,y(-m/1.2)); ctx.stroke();
  ctx.setLineDash([]);

  const line = (key, color, dash) => {
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.setLineDash(dash ? [5,4] : []);
    ctx.beginPath();
    hist.forEach((p,i) => { const yy = y(p[key]); i ? ctx.lineTo(x(i),yy) : ctx.moveTo(x(i),yy); });
    ctx.stroke(); ctx.setLineDash([]);
  };
  line('set', '#8b98a9', true);
  line('vel', '#4da3ff', false);

  ctx.fillStyle = '#8b98a9'; ctx.font = '12px sans-serif';
  ctx.fillText('+' + m.toFixed(2), 10, 16);
  ctx.fillText('-' + m.toFixed(2), 10, h - 6);
  ctx.fillStyle = '#4da3ff'; ctx.fillText('vel', w - 66, 16);
  ctx.fillStyle = '#8b98a9'; ctx.fillText('set', w - 36, 16);
}

$('btnStart').onclick = () => send('start');
$('btnStop').onclick = () => send('stop');
$('btnClear').onclick = () => send('clear');
$('btnReconnect').onclick = () => send('reconnect');
$('btnVel').onclick = () => send('vel', parseFloat($('velIn').value));
$('btnRamp').onclick = () => send('ramp', parseFloat($('velIn').value));
$('btnPos').onclick = () => send('pos', parseFloat($('posIn').value));
$('btnRel').onclick = () => send('rel', parseFloat($('posIn').value));
$('btnCalMotor').onclick = () => { if (confirm('开始电机标定？')) send('cal_motor'); };
$('btnCalEnc').onclick = () => {
  if (confirm('编码器偏置标定：电机会转动约 1.1 圈，确认周围无障碍？')) send('cal_enc');
};
$('btnSave').onclick = () => { if (confirm('保存配置并重启？')) send('save'); };

refresh();
setInterval(refresh, 500);
window.addEventListener('resize', draw);
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
        else:
            self._send(404, "{}")

    def do_POST(self):
        if self.path.startswith("/api/cmd"):
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                body = {}
            self._send(200, json.dumps(run_cmd(body.get("action"), body.get("value"))))
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
            print("（打开页面后可以点“重连”）")

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
