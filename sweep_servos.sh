#!/usr/bin/env bash
# 全サーボを一気に 0°(min) -> 180°(max) -> 90°(neutral) へ動かすスクリプト
# 使い方:
#   ./sweep_servos.sh          # シリアル経由で sweep コマンドを送る (未書き込みなら --upload を付ける)
#   ./sweep_servos.sh --upload # ビルド & 書き込みしてから実行
#   ./sweep_servos.sh min      # 全サーボを 0° に固定 (min / max / neutral / sweep / check など指定可)

set -e
cd "$(dirname "$0")"

CMD="sweep"
UPLOAD=0

for arg in "$@"; do
  if [ "$arg" = "--upload" ] || [ "$arg" = "-u" ]; then
    UPLOAD=1
  else
    CMD="$arg"
  fi
done

if [ "$UPLOAD" -eq 1 ]; then
  echo "[1/2] Building and uploading firmware..."
  pio run -t upload
fi

# シリアルポート自動検出 (/dev/ttyACM* または /dev/ttyUSB*)
PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1 || true)
if [ -z "$PORT" ]; then
  echo "エラー: Arduino のシリアルポート (/dev/ttyACM* または /dev/ttyUSB*) が見つかりません。" >&2
  echo "USBケーブルが接続されているか確認してください。" >&2
  exit 1
fi

echo "Port: $PORT  Command: $CMD"

python3 - "$PORT" "$CMD" <<'EOF'
import sys, time, termios

port, cmd = sys.argv[1], sys.argv[2]

# 115200 bps, 8N1 でポートを開く (開いた瞬間に Arduino Uno は自動リセットされる)
f = open(port, "r+b", buffering=0)
attrs = termios.tcgetattr(f)
attrs[0] = 0
attrs[1] = 0
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0
attrs[4] = termios.B115200
attrs[5] = termios.B115200
termios.tcsetattr(f, termios.TCSANOW, attrs)

print("Waiting 2.0s for Arduino boot...")
time.sleep(2.0)
termios.tcflush(f, termios.TCIFLUSH)

print(f"Sending: {cmd}")
f.write((cmd + "\n").encode())

# 応答を約3.5秒間読み取る
end = time.time() + 3.5
import select
while time.time() < end:
    r, _, _ = select.select([f], [], [], 0.1)
    if r:
        data = f.read(256)
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()
f.close()
EOF
