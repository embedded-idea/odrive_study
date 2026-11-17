"""OdrvStudio - odrive-interface.yaml 解析,生成参数树模型。

参考 SimpleFOCStudio 的 device.json 驱动树形浏览的思路,
这里直接解析 ODrive 固件仓库自带的 odrive-interface.yaml。
"""

import os

import yaml

DEFAULT_SPEC = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "odrive-interface.yaml"
)


class Attr:
    """参数树节点。children 为空表示叶子(可读值)。"""

    __slots__ = ("name", "path", "type", "mode", "unit", "brief",
                 "flags", "bits", "children", "is_func")

    def __init__(self, name, path, type_="object", mode="rw", unit="",
                 brief="", flags=None, bits=None, children=None, is_func=False):
        self.name = name
        self.path = path
        self.type = type_
        self.mode = mode          # "r" / "rw"
        self.unit = unit
        self.brief = brief
        self.flags = flags or []  # 错误位名称(bitmask 用)
        self.bits = bits or {}    # name -> bit
        self.children = children or []
        self.is_func = is_func

    @property
    def writable(self):
        return self.mode == "rw" and not self.children and not self.is_func

    @property
    def is_error(self):
        return bool(self.flags)


def _split_type(raw):
    """'readonly float32' -> ('r', 'float32'); 'float32' -> ('rw', 'float32')"""
    parts = str(raw or "").split()
    if parts and parts[0] == "readonly":
        return "r", " ".join(parts[1:]) or "object"
    return "rw", str(raw or "object")


def _resolve_iface(interfaces, type_):
    if type_ in interfaces:
        return interfaces[type_]
    qualified = "ODrive." + type_
    if qualified in interfaces:
        return interfaces[qualified]
    return None


def _iface_attrs(interfaces, name, depth=0):
    """取接口属性,合并其 implements 的基接口(自身优先)。"""
    iface = interfaces.get(name)
    if iface is None or depth > 4:
        return {}
    attrs = dict(iface.get("attributes") or {})
    base = iface.get("implements")
    if isinstance(base, str):
        for k, v in _iface_attrs(interfaces, base, depth + 1).items():
            attrs.setdefault(k, v)
    return attrs


def _walk(attrs, prefix, interfaces, depth=0):
    out = []
    for name, a in (attrs or {}).items():
        if not isinstance(a, dict):        # 简写: `sampling: TaskTimer`
            a = {"type": a}
        if depth > 8:                      # 防御循环引用
            continue
        if "in" in a or "out" in a:        # fibre 函数,不是可读属性
            out.append(Attr(name, prefix + name, "function", "r",
                            brief=a.get("brief", a.get("doc", "")), is_func=True))
            continue

        mode, type_ = _split_type(a.get("type", ""))
        node = Attr(
            name, prefix + name, type_, mode,
            unit=a.get("unit", ""),
            brief=a.get("brief", "") or a.get("doc", ""),
        )
        if "flags" in a:  # 错误位:按声明顺序补 bit 序号
            bit = 0
            for fname, fa in (a["flags"] or {}).items():
                b = int(fa.get("bit", bit)) if isinstance(fa, dict) else bit
                node.flags.append(fname)
                node.bits[fname] = b
                bit = b + 1
        kids = a.get("attributes")
        impl = a.get("implements")
        if kids is not None or isinstance(impl, str):
            merged = {}
            if isinstance(impl, str):
                merged.update(_iface_attrs(interfaces, impl, 1))
            merged.update(kids or {})
            node.children = _walk(merged, node.path + ".", interfaces, depth + 1)
        else:
            iface = _resolve_iface(interfaces, type_)
            if iface is not None and iface.get("attributes"):
                node.children = _walk(iface["attributes"], node.path + ".",
                                      interfaces, depth + 1)
        out.append(node)
    return out


def load_spec(path=None):
    """返回设备顶层 Attr 列表(以具体板卡接口为根,如 ODrive3)。"""
    path = os.path.abspath(path or DEFAULT_SPEC)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    interfaces = data.get("interfaces", {})
    root_name = "ODrive3" if "ODrive3" in interfaces else "ODrive"
    root = _iface_attrs(interfaces, root_name)
    return _walk(root, "", interfaces)


def decode_bits(mask, node):
    """把错误掩码翻译成 'NAME1 | NAME2'。"""
    if not mask:
        return "NONE"
    names = [n for n in node.flags if mask & (1 << node.bits[n])]
    return " | ".join(names) if names else "0x%x" % mask


def build_index(roots):
    """path -> Attr 的扁平索引。"""
    idx = {}

    def rec(nodes):
        for n in nodes:
            idx[n.path] = n
            rec(n.children)

    rec(roots)
    return idx


def writable_leaves(node):
    """收集节点子树内全部可写叶子 Attr(保持 yaml 顺序)。"""
    out = []

    def rec(n):
        if n.children:
            for c in n.children:
                rec(c)
        elif n.writable:
            out.append(n)

    rec(node)
    return out
