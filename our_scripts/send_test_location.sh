#!/bin/bash
# Sends a test destination to the Ouranos bridge via TCP socket.
# Usage: ./send_destination.sh [test_number]
#   test_number 1 → x=0, y=-10, z=-5  (10m west, 5m up)
#   test_number 2 → x=-5, y=-10, z=-5  (5m south, 10m west, 5m up)
#   (default: 1)

TEST=${1:-1}

case $TEST in
    1)
        LAT="37.412173"
        LON="-121.998991"
        ALT="43.0"
        EXPECTED="x=0, y=-10, z=-5"
        ;;
    2)
        LAT="37.412128"
        LON="-121.998991"
        ALT="43.0"
        EXPECTED="x=-5, y=-10, z=-5"
        ;;
    *)
        echo "Usage: $0 [1|2]"
        echo "  1 → x=0, y=-10, z=-5"
        echo "  2 → x=-5, y=-10, z=-5"
        exit 1
        ;;
esac

echo "═══════════════════════════════════════"
echo "  Sending test destination #$TEST"
echo "  Expected NED: $EXPECTED"
echo "═══════════════════════════════════════"

python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 9091))
s.sendall(b'<destination>{\"lat\":$LAT,\"lon\":$LON,\"alt\":$ALT}\n')
s.shutdown(socket.SHUT_WR)
import time; time.sleep(0.5)
s.close()
print(f'Sent GPS: lat=$LAT, lon=$LON, alt=$ALT')
print(f'Expected NED: $EXPECTED')
"
