#!/usr/bin/env python
import subprocess
import re

def getCurrentPose():
    try:
        cmd = [
            "timeout", "3",
            "rosrun", "tf", "tf_echo", "base_link", "Link6"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        out = result.stdout + result.stderr

        # ✅ 适配你真实输出的格式
        t = re.search(
            r"Translation:\s*\[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\]",
            out
        )
        r = re.search(
            r"in Quaternion\s*\[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*\]",
            out
        )

        if t and r:
            return (
                {"x": float(t[1]), "y": float(t[2]), "z": float(t[3])},
                {"x": float(r[1]), "y": float(r[2]), "z": float(r[3]), "w": float(r[4])}
            )
        else:
            print("❌ 正则未匹配")
            print(out)

    except Exception as e:
        print("❌ 异常:", e)

    return None, None


if __name__ == "__main__":
    pos, ori = getCurrentPose()
    if pos:
        print("✅ 当前末端位姿：")
        print("位置:", pos)
        print("姿态:", ori)
