#!/usr/bin/env python3
"""仿真相机话题的网页取流视图。

在容器内（Jazzy）运行，浏览器在宿主机打开 http://localhost:<port>。
跨发行版（宿主 Humble ↔ 容器 Jazzy）订阅不通，所以取流必须放在容器里。

    python3 tools/sim/image_view_server.py [--port 8090]

页面左上下拉框列出当前所有 sensor_msgs/Image 与 CompressedImage 话题，
选中即开始取流；订阅使用 SensorDataQoS（BEST_EFFORT），与相机发布端一致。
"""

import argparse
import json
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")


class Feed:
    """一条已订阅的图像话题：保留最新一帧和到达速率。"""

    __slots__ = ("topic", "type_name", "msg", "stamp", "seq", "count", "arrivals", "sub")

    def __init__(self, topic, type_name):
        self.topic = topic
        self.type_name = type_name
        self.msg = None
        self.stamp = 0.0
        self.seq = 0
        self.count = 0
        self.arrivals = deque(maxlen=64)
        self.sub = None

    def hz(self):
        now = time.time()
        while self.arrivals and now - self.arrivals[0] > 2.0:
            self.arrivals.popleft()
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0


class Hub:
    """按话题懒创建订阅；只有被页面选中过的话题才会订阅。"""

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._feeds = {}

    def type_of(self, topic):
        for name, types in self._node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        return None

    def image_topics(self):
        result = []
        for name, types in self._node.get_topic_names_and_types():
            if types and types[0] in IMAGE_TYPES:
                feed = self._feeds.get(name)
                info = {
                    "topic": name,
                    "type": types[0],
                    "hz": round(feed.hz(), 1) if feed else 0.0,
                }
                if feed and feed.msg is not None and isinstance(feed.msg, Image):
                    info.update(
                        width=feed.msg.width,
                        height=feed.msg.height,
                        encoding=feed.msg.encoding,
                        frame_id=feed.msg.header.frame_id,
                    )
                result.append(info)
        return sorted(result, key=lambda item: item["topic"])

    def acquire(self, topic):
        with self._lock:
            feed = self._feeds.get(topic)
            if feed is not None:
                return feed
            type_name = self.type_of(topic)
            if type_name not in IMAGE_TYPES:
                return None
            feed = Feed(topic, type_name)
            msg_type = CompressedImage if type_name.endswith("CompressedImage") else Image
            feed.sub = self._node.create_subscription(
                msg_type,
                topic,
                lambda msg, f=feed: self._on_msg(f, msg),
                qos_profile_sensor_data,
            )
            self._feeds[topic] = feed
            return feed

    @staticmethod
    def _on_msg(feed, msg):
        feed.msg = msg
        feed.stamp = time.time()
        feed.seq += 1
        feed.count += 1
        feed.arrivals.append(feed.stamp)

    def snapshot(self, feed):
        with self._lock:
            return feed.seq, feed.msg


_DTYPE = {
    "rgb8": np.uint8,
    "bgr8": np.uint8,
    "rgba8": np.uint8,
    "bgra8": np.uint8,
    "mono8": np.uint8,
    "8uc1": np.uint8,
    "mono16": np.uint16,
    "16uc1": np.uint16,
    "rgb16": np.uint16,
    "bgr16": np.uint16,
    "32fc1": np.float32,
}


def _to_bgr(msg):
    if isinstance(msg, CompressedImage):
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    enc = (msg.encoding or "").lower()
    dtype = _DTYPE.get(enc)
    if dtype is None:
        return None
    h, w = msg.height, msg.width
    ch = 4 if enc.startswith(("rgba", "bgra")) else 3 if enc.startswith(("rgb", "bgr")) else 1
    row_items = msg.step // np.dtype(dtype).itemsize
    raw = np.frombuffer(bytes(msg.data), dtype)
    if raw.size < h * row_items:
        return None
    plane = raw.reshape(h, row_items)[:, : w * ch]

    if ch > 1:
        arr = plane.reshape(h, w, ch)
        if enc.startswith("rgb"):
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if ch == 4:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return arr

    gray = plane.reshape(h, w)
    if dtype == np.uint8:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if dtype == np.float32:
        gray = np.nan_to_num(gray, nan=0.0, posinf=0.0, neginf=0.0)
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    else:
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    return cv2.cvtColor(gray.astype(np.uint8), cv2.COLOR_GRAY2BGR)


def encode_jpeg(msg, quality):
    try:
        bgr = _to_bgr(msg)
    except Exception:  # noqa: BLE001 - 坏帧不应打断取流
        return None
    if bgr is None or bgr.size == 0:
        return None
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Thinker-Skill 相机视图</title>
<style>
  :root { color-scheme: dark; --bg:#101318; --panel:#181d24; --line:#2a323d;
          --text:#d8dee9; --muted:#8b97a8; --accent:#4aa8ff; }
  * { box-sizing: border-box; }
  body { margin:0; height:100vh; display:flex; flex-direction:column;
         background:var(--bg); color:var(--text);
         font:14px/1.5 system-ui, "Segoe UI", "Noto Sans SC", sans-serif; }
  header { display:flex; align-items:center; gap:12px; padding:10px 16px;
           background:var(--panel); border-bottom:1px solid var(--line); flex-wrap:wrap; }
  header h1 { font-size:15px; margin:0 8px 0 0; font-weight:600; }
  select, input, button { background:#0e1218; color:var(--text); border:1px solid var(--line);
           border-radius:6px; padding:6px 8px; font:inherit; }
  select { min-width:340px; }
  button { cursor:pointer; }
  button:hover { border-color:var(--accent); }
  label { color:var(--muted); display:flex; align-items:center; gap:6px; }
  #status { margin-left:auto; color:var(--muted); font-variant-numeric:tabular-nums; }
  main { flex:1; min-height:0; display:flex; align-items:center; justify-content:center;
         padding:12px; background:#0b0e12; }
  #view { max-width:100%; max-height:100%; object-fit:contain; border-radius:4px;
          background:#0b0e12; display:none; }
  #hint { color:var(--muted); text-align:center; max-width:520px; }
  #hint code { color:var(--text); }
</style>
</head>
<body>
<header>
  <h1>相机视图</h1>
  <select id="topics"><option value="">（没有图像话题）</option></select>
  <button id="refresh" title="刷新话题列表">刷新</button>
  <label>帧率
    <select id="fps">
      <option>5</option><option>10</option><option selected>15</option><option>20</option>
    </select>
  </label>
  <label>画质 <input id="quality" type="range" min="30" max="95" value="80"
        style="width:100px"> <span id="qv">80</span></label>
  <span id="status">未选择话题</span>
</header>
<main>
  <img id="view" alt="camera">
  <div id="hint">选择一个相机话题开始取流。<br>
    话题列表来自当前 ROS 图（容器内 Jazzy）。</div>
</main>
<script>
const $ = id => document.getElementById(id);
const topicsEl = $("topics"), view = $("view"), hint = $("hint"), status = $("status");
let feeds = [];

function start() {
  const topic = topicsEl.value;
  if (!topic) { view.style.display = "none"; hint.style.display = "block"; return; }
  const meta = feeds.find(f => f.topic === topic);
  const q = new URLSearchParams({topic, fps: $("fps").value, quality: $("quality").value});
  view.src = "/stream?" + q.toString() + "&t=" + Date.now();
  view.style.display = "block";
  hint.style.display = "none";
  const dim = meta && meta.width ? meta.width + "×" + meta.height + " " + meta.encoding : "";
  status.textContent = (dim ? dim + " · " : "") + (meta ? meta.hz + " Hz" : "");
  const u = new URL(location.href);
  u.searchParams.set("topic", topic);
  history.replaceState(null, "", u);
}

async function loadTopics(initial) {
  if (!initial && document.activeElement === topicsEl) return;  // 别打断正在选话题的用户
  try {
    const r = await fetch("/api/topics", {cache: "no-store"});
    feeds = await r.json();
  } catch (e) { status.textContent = "无法连接取流服务"; return; }
  const prev = topicsEl.value || new URLSearchParams(location.search).get("topic") || "";
  topicsEl.innerHTML = "";
  if (!feeds.length) {
    const o = document.createElement("option");
    o.value = ""; o.textContent = "（没有图像话题）"; topicsEl.append(o);
    if (initial) { view.style.display = "none"; hint.style.display = "block"; }
    return;
  }
  for (const f of feeds) {
    const o = document.createElement("option");
    o.value = f.topic;
    o.textContent = f.topic + "  [" + f.type.split("/").pop() + "]";
    topicsEl.append(o);
  }
  const keep = feeds.some(f => f.topic === prev) ? prev
             : feeds.find(f => /color\/image_raw$/.test(f.topic)) ? feeds.find(f => /color\/image_raw$/.test(f.topic)).topic
             : feeds[0].topic;
  topicsEl.value = keep;
  if (initial) start();
  const meta = feeds.find(f => f.topic === topicsEl.value);
  if (meta) {
    const dim = meta.width ? meta.width + "×" + meta.height + " " + meta.encoding + " · " : "";
    status.textContent = dim + meta.hz + " Hz";
  }
}

topicsEl.addEventListener("change", start);
$("fps").addEventListener("change", start);
$("quality").addEventListener("input", e => { $("qv").textContent = e.target.value; start(); });
$("refresh").addEventListener("click", () => loadTopics(false));
loadTopics(true);
setInterval(() => loadTopics(false), 4000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    hub = None

    def log_message(self, fmt, *args):  # 保持控制台干净
        pass

    def _send(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        if parsed.path == "/":
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return

        if parsed.path == "/api/topics":
            body = json.dumps(self.hub.image_topics(), ensure_ascii=False).encode("utf-8")
            self._send(200, body, "application/json; charset=utf-8")
            return

        if parsed.path in ("/stream", "/snapshot"):
            topic = (query.get("topic") or [""])[0]
            quality = _int_arg(query, "quality", 80, 10, 95)
            feed = self.hub.acquire(topic)
            if feed is None:
                self._send(404, b"unknown image topic", "text/plain; charset=utf-8")
                return
            if parsed.path == "/snapshot":
                self._snapshot(feed, quality)
            else:
                self._stream(feed, _int_arg(query, "fps", 15, 1, 30), quality)
            return

        self._send(404, b"not found", "text/plain; charset=utf-8")

    def _snapshot(self, feed, quality):
        deadline = time.time() + 5.0
        while time.time() < deadline:
            seq, msg = self.hub.snapshot(feed)
            if msg is not None:
                jpg = encode_jpeg(msg, quality)
                if jpg:
                    self._send(200, jpg, "image/jpeg")
                    return
            time.sleep(0.05)
        self._send(504, b"no frame", "text/plain; charset=utf-8")

    def _stream(self, feed, fps, quality):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_seq = -1
        delay = 1.0 / fps
        try:
            while True:
                seq, msg = self.hub.snapshot(feed)
                if msg is not None and seq != last_seq:
                    jpg = encode_jpeg(msg, quality)
                    if jpg:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpg)).encode() + b"\r\n\r\n"
                        )
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    last_seq = seq
                time.sleep(delay)
        except (BrokenPipeError, ConnectionResetError):
            pass


def _int_arg(query, name, default, low, high):
    try:
        value = int((query.get(name) or [default])[0])
    except (TypeError, ValueError):
        return default
    return max(low, min(high, value))


def main(argv=None):
    parser = argparse.ArgumentParser(description="相机话题网页取流（在仿真容器内运行）")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("sim_image_view")
    hub = Hub(node)
    Handler.hub = hub

    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as exc:
        print(f"端口 {args.port} 不可用（{exc.strerror}）：先停掉旧进程，或用 --port 换端口。",
              file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 2
    server.daemon_threads = True
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f"相机视图: http://localhost:{args.port}", flush=True)
    print("Ctrl-C 退出；页面可切换话题、帧率与画质。", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
