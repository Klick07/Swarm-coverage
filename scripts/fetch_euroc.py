"""Download EuRoC sequences (ASL format) to ~/data/euroc.

  python fetch_euroc.py                                   # MH_01_easy MH_02_easy MH_03_medium
  python fetch_euroc.py vicon_room1 V1_01_easy V1_02_medium V1_03_difficult

ETH's Research Collection blocks scripted downloads, so this reads the unofficial
Hugging Face mirror GlowBond/EuRoC_MAV_Dataset (machine_hall.zip 12.7 GB, vicon_room1.zip
and vicon_room2.zip 6.0 GB each). HTTP range reads pull out only the inner ASL zips
we need, CRC-checked by zipfile. Needs: pip install remotezip==0.12.3
"""
import os
import subprocess
import sys
import zipfile

from remotezip import RemoteZip

MIRROR = "https://huggingface.co/datasets/GlowBond/EuRoC_MAV_Dataset/resolve/main/{}.zip"
OUT = os.path.expanduser("~/data/euroc")

location = sys.argv[1] if len(sys.argv) > 1 else "machine_hall"
sequences = sys.argv[2:] or ["MH_01_easy", "MH_02_easy", "MH_03_medium"]
for seq in sequences:
    dest = os.path.join(OUT, seq)
    if os.path.isdir(os.path.join(dest, "mav0")):
        print(seq, "already present", flush=True)
        continue
    print("fetching", seq, flush=True)
    with RemoteZip(MIRROR.format(location)) as z:
        inner = z.extract(f"{location}/{seq}/{seq}.zip", path=OUT)
    os.makedirs(dest, exist_ok=True)
    with zipfile.ZipFile(inner) as iz:
        bad = iz.testzip()
        assert bad is None, f"corrupt member {bad}"
        iz.extractall(dest)
    os.remove(inner)
    print(seq, "done:", subprocess.run(["du", "-sh", dest], capture_output=True, text=True).stdout.strip(), flush=True)
print("FETCH_DONE", flush=True)
