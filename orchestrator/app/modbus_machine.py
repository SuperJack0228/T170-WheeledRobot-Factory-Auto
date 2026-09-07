"""机床 Modbus TCP：编排台主动读/写保持寄存器（功能码 03/06）。"""

from __future__ import annotations

import socket
import struct
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "config" / "machine_modbus.yaml"

LogFn = Callable[[str, str, str], None]


@dataclass(frozen=True)
class SignalSpec:
    code: int
    host_key: str
    ip: str
    addr: int
    direction: str
    name: str


@dataclass
class MachineModbusConfig:
    port: int = 502
    unit_id: int = 1
    poll_ms: int = 200
    connect_timeout_ms: int = 3000
    default_wait_sec: float = 120.0
    hosts: dict[str, str] | None = None
    signals: dict[int, SignalSpec] | None = None


_cfg_lock = threading.Lock()
_cfg_cache: MachineModbusConfig | None = None
_tid_lock = threading.Lock()
_tid = 0


def _default_config() -> MachineModbusConfig:
    hosts = {"m88": "192.168.1.88", "m89": "192.168.1.89"}
    raw = {
        10: (2100, "read", "机床可以上料"),
        20: (2200, "read", "机床可以下料"),
        35: (2150, "write", "到预备上料点-锁定传送带"),
        15: (2150, "write", "上料完成"),
        45: (2250, "write", "到预备下料点-锁定传送带"),
        25: (2250, "write", "下料完成"),
    }
    signals = {
        code: SignalSpec(code, "", "", addr, direction, name)
        for code, (addr, direction, name) in raw.items()
    }
    return MachineModbusConfig(hosts=hosts, signals=signals)


def _parse_yaml_simple(text: str) -> dict[str, Any]:
    """只够本文件用的缩进 yaml（不引第三方）。"""
    root: dict[str, Any] = {}
    stack: list[tuple[int, Any]] = [(-1, root)]
    pending_key: str | None = None
    pending_indent = 0

    def parse_scalar(raw: str) -> Any:
        s = raw.strip()
        if s.startswith("#"):
            return None
        if "#" in s:
            s = s.split("#", 1)[0].strip()
        if not s:
            return None
        if (s.startswith('"') and s.endswith('"')) or (s.startswith("'") and s.endswith("'")):
            return s[1:-1]
        try:
            if "." in s:
                return float(s)
            return int(s)
        except ValueError:
            return s

    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        while stack and indent <= stack[-1][0] and not (
            pending_key is not None and indent > pending_indent and stack[-1][0] == pending_indent
        ):
            if pending_key is not None and indent <= pending_indent:
                pending_key = None
            if indent <= stack[-1][0]:
                stack.pop()
            else:
                break
        parent = stack[-1][1]
        if ":" not in stripped:
            continue
        key, rest = stripped.split(":", 1)
        key = key.strip()
        rest = rest.strip()
        if rest.startswith("#"):
            rest = ""
        elif "#" in rest:
            rest = rest.split("#", 1)[0].strip()
        if rest == "":
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
            pending_key = key
            pending_indent = indent
        else:
            parent[key] = parse_scalar(rest)
            pending_key = None
    return root


def load_config(force: bool = False) -> MachineModbusConfig:
    global _cfg_cache
    with _cfg_lock:
        if _cfg_cache is not None and not force:
            return _cfg_cache
        cfg = _default_config()
        if CONFIG_PATH.is_file():
            data = _parse_yaml_simple(CONFIG_PATH.read_text(encoding="utf-8"))
            if data.get("port"):
                cfg.port = int(data["port"])
            if data.get("unit_id") is not None:
                cfg.unit_id = int(data["unit_id"])
            if data.get("poll_ms"):
                cfg.poll_ms = int(data["poll_ms"])
            if data.get("connect_timeout_ms"):
                cfg.connect_timeout_ms = int(data["connect_timeout_ms"])
            if data.get("default_wait_sec"):
                cfg.default_wait_sec = float(data["default_wait_sec"])
            hosts = data.get("hosts")
            if isinstance(hosts, dict):
                cfg.hosts = {str(k): str(v) for k, v in hosts.items()}
            sigs = data.get("signals")
            if isinstance(sigs, dict):
                parsed: dict[int, SignalSpec] = {}
                for k, v in sigs.items():
                    if not isinstance(v, dict):
                        continue
                    code = int(k)
                    host_key = str(v.get("host") or "")
                    ip = cfg.hosts.get(host_key, host_key) if host_key else ""
                    parsed[code] = SignalSpec(
                        code=code,
                        host_key=host_key,
                        ip=str(ip),
                        addr=int(v.get("addr") or 0),
                        direction=str(v.get("dir") or "read"),
                        name=str(v.get("name") or f"信号{code}"),
                    )
                if parsed:
                    cfg.signals = parsed
        _cfg_cache = cfg
        return cfg


_last_ready_lock = threading.Lock()
_last_ready: dict[str, Any] | None = None


def last_ready() -> dict[str, Any] | None:
    with _last_ready_lock:
        return dict(_last_ready) if _last_ready else None


def _set_last_ready(ip: str, host_key: str, signal: int, addr: int) -> None:
    global _last_ready
    with _last_ready_lock:
        _last_ready = {"ip": ip, "host": host_key, "signal": int(signal), "addr": int(addr)}


def poll_targets(signal: int, addr: int | None = None) -> list[SignalSpec]:
    """等待时要读的许可寄存器：默认 10@2100 和 20@2200 都读。"""
    cfg = load_config()
    sigs = cfg.signals or {}
    if addr not in (None, "", 0):
        spec = sigs.get(int(signal))
        name = spec.name if spec else f"信号{signal}"
        return [SignalSpec(int(signal), "", "", int(addr), "read", name)]
    reads = [s for s in sigs.values() if s.direction == "read"]
    if not reads:
        spec = sigs.get(int(signal))
        if spec is None:
            raise ValueError(f"未知机床信号 {signal}")
        return [spec]
    want = int(signal)
    ordered = [s for s in reads if s.code == want] + [s for s in reads if s.code != want]
    return ordered


def wait_hosts(host: str = "") -> list[tuple[str, str]]:
    """[(host_key, ip), ...]。未指定覆盖时轮询 yaml 里全部主机。"""
    cfg = load_config()
    if host:
        ip = resolve_host_ip(host)
        key = host if (cfg.hosts and host in cfg.hosts) else ip
        return [(key, ip)]
    items = list((cfg.hosts or {}).items())
    if not items:
        raise ValueError("config/machine_modbus.yaml 未配置 hosts")
    seen: set[str] = set()
    out: list[tuple[str, str]] = []
    for key, ip in items:
        ip = str(ip)
        if ip in seen:
            continue
        seen.add(ip)
        out.append((str(key), ip))
    return out


def signal_spec(code: int) -> SignalSpec:
    cfg = load_config()
    spec = (cfg.signals or {}).get(int(code))
    if spec is None:
        raise ValueError(f"未知机床信号 {code}，请在 config/machine_modbus.yaml 里登记")
    return spec


def resolve_host_ip(host: str) -> str:
    host = (host or "").strip()
    if not host:
        raise ValueError("未指定主机")
    cfg = load_config()
    if cfg.hosts and host in cfg.hosts:
        return cfg.hosts[host]
    fallback = {"load": "192.168.1.88", "unload": "192.168.1.89"}
    if host in fallback:
        return fallback[host]
    return host


def _next_tid() -> int:
    global _tid
    with _tid_lock:
        _tid = (_tid + 1) & 0xFFFF
        if _tid == 0:
            _tid = 1
        return _tid


class ModbusTcpClient:
    def __init__(self, ip: str, port: int = 502, unit_id: int = 1, timeout_s: float = 3.0):
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def connect(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.timeout_s)
        sock.connect((self.ip, self.port))
        self._sock = sock

    def _transact(self, pdu: bytes) -> bytes:
        if self._sock is None:
            self.connect()
        assert self._sock is not None
        tid = _next_tid()
        mbap = struct.pack(">HHHB", tid, 0, 1 + len(pdu), self.unit_id)
        self._sock.sendall(mbap + pdu)
        hdr = self._recv_exact(7)
        rtid, proto, length = struct.unpack(">HHH", hdr[:6])
        if proto != 0:
            raise RuntimeError("协议标识不是 Modbus")
        if rtid != tid:
            raise RuntimeError("事务标识不匹配")
        if length < 2 or length > 253:
            raise RuntimeError("MBAP 长度异常")
        body = self._recv_exact(length - 1)
        if not body:
            raise RuntimeError("响应 PDU 为空")
        if body[0] & 0x80:
            ex = body[1] if len(body) > 1 else 0
            raise RuntimeError(f"从站异常码 0x{ex:02X}")
        return body

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise RuntimeError("对端关闭连接")
            buf.extend(chunk)
        return bytes(buf)

    def read_holding(self, addr: int, quantity: int = 1) -> list[int]:
        pdu = struct.pack(">BHH", 0x03, addr, quantity)
        try:
            rsp = self._transact(pdu)
        except Exception:
            self.close()
            raise
        if rsp[0] != 0x03 or len(rsp) < 2:
            raise RuntimeError("读响应异常")
        byte_count = rsp[1]
        if byte_count != quantity * 2 or len(rsp) < 2 + byte_count:
            raise RuntimeError("寄存器数据不完整")
        values = []
        for i in range(quantity):
            values.append(struct.unpack(">H", rsp[2 + i * 2 : 4 + i * 2])[0])
        return values

    def write_single(self, addr: int, value: int) -> None:
        value = int(value) & 0xFFFF
        pdu = struct.pack(">BHH", 0x06, addr, value)
        try:
            rsp = self._transact(pdu)
        except Exception:
            self.close()
            raise
        if rsp[0] != 0x06 or len(rsp) < 5:
            raise RuntimeError("写响应异常")
        got_addr, got_val = struct.unpack(">HH", rsp[1:5])
        if got_addr != addr or got_val != value:
            raise RuntimeError("写响应与请求不一致")


def _client_for(ip: str) -> ModbusTcpClient:
    cfg = load_config()
    return ModbusTcpClient(
        ip,
        port=cfg.port,
        unit_id=cfg.unit_id,
        timeout_s=max(0.2, cfg.connect_timeout_ms / 1000.0),
    )


def wait_signal(
    signal: int,
    timeout_sec: float,
    *,
    log: LogFn,
    check_abort: Callable[[], None],
    sleep_s: Callable[[float], None],
    host: str = "",
    addr: int | None = None,
) -> int:
    """轮询 88/89 的上料(10@2100)和下料(20@2200)许可，直到目标信号出现。"""
    cfg = load_config()
    spec = (cfg.signals or {}).get(int(signal))
    if spec and spec.direction != "read":
        raise ValueError(f"信号 {signal} 是写出项，不能用来等待")
    name = spec.name if spec else f"信号{signal}"
    hosts = wait_hosts(host)
    targets = poll_targets(int(signal), addr)
    if not targets or any(t.addr <= 0 for t in targets):
        raise ValueError("等待信号需要有效寄存器地址")

    timeout_sec = float(timeout_sec if timeout_sec and timeout_sec > 0 else cfg.default_wait_sec)
    poll_s = max(0.05, cfg.poll_ms / 1000.0)
    ips = ", ".join(ip for _, ip in hosts)
    regs = ", ".join(f"{t.addr}→{t.code}({t.name})" for t in targets)
    log("INFO", "modbus", f"轮询 {ips}:{cfg.port} 保持寄存器 {regs}，等待信号 {signal}（{name}）")

    clients = {ip: _client_for(ip) for _, ip in hosts}
    last_vals: dict[tuple[str, int], int | None] = {}
    last_errs: dict[tuple[str, int], str] = {}
    elapsed = 0.0
    try:
        while True:
            check_abort()
            for host_key, ip in hosts:
                client = clients[ip]
                for tgt in targets:
                    key = (ip, tgt.addr)
                    try:
                        val = client.read_holding(tgt.addr, 1)[0]
                        last_errs.pop(key, None)
                        if last_vals.get(key) != val:
                            log("INFO", "modbus", f"{ip} reg[{tgt.addr}] = {val}")
                            last_vals[key] = val
                        if val == tgt.code:
                            msg = f"收到信号 {tgt.code} {tgt.name}  ({ip} reg={tgt.addr} val={val})"
                            if tgt.code == int(signal):
                                log("INFO", "modbus", msg)
                                print(f"[modbus] {msg}", flush=True)
                                _set_last_ready(ip, host_key, tgt.code, tgt.addr)
                                return val
                            log("INFO", "modbus", f"看到 {msg}，本步继续等 {signal}（{name}）")
                    except Exception as exc:  # noqa: BLE001
                        client.close()
                        err = str(exc)
                        if last_errs.get(key) != err:
                            log("WARN", "modbus", f"读 {ip}:{tgt.addr} 失败: {err}，继续重试")
                            last_errs[key] = err
            if elapsed >= timeout_sec:
                snap = ", ".join(
                    f"{ip}[{reg}]={val}" for (ip, reg), val in last_vals.items()
                ) or "无读数"
                raise TimeoutError(
                    f"等待信号 {signal}（{name}）超时 {timeout_sec:.0f}s，最后={snap}"
                )
            sleep_s(poll_s)
            elapsed += poll_s
    finally:
        for client in clients.values():
            client.close()


def send_value(
    value: int,
    *,
    log: LogFn,
    host: str = "",
    addr: int | None = None,
) -> None:
    """把 value 写入对应寄存器。未指定主机时写最近等到许可的那台。"""
    cfg = load_config()
    value = int(value)
    spec = (cfg.signals or {}).get(value)
    register = 0
    name = f"自定义 {value}"
    if addr not in (None, ""):
        register = int(addr)
    elif spec:
        register = spec.addr
    if spec:
        name = spec.name
        if spec.direction != "write" and not host and addr in (None, ""):
            raise ValueError(f"信号 {value} 是机床发出的就绪位，不能当写出指令")
    if register <= 0:
        raise ValueError("发送需要寄存器（或填写已知信号 15/25/35/45）")

    ips: list[str] = []
    if host:
        ips = [resolve_host_ip(host)]
    else:
        ready = last_ready()
        if ready and ready.get("ip"):
            ips = [str(ready["ip"])]
        else:
            ips = [ip for _, ip in wait_hosts("")]
    if not ips:
        raise ValueError("发送需要主机（或先等待上/下料许可）")

    for ip in ips:
        client = _client_for(ip)
        try:
            client.connect()
            client.write_single(register, value)
        finally:
            client.close()
        log("INFO", "modbus", f"已写 {ip}:{cfg.port} reg[{register}] = {value}（{name}）")
        print(f"[modbus] 已发送 {value} {name}  -> {ip}:{register}", flush=True)
