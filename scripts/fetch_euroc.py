"""Download EuRoC MH_01_easy, MH_02_easy, MH_03_medium (ASL format) to ~/data/euroc.

ETH's Research Collection blocks scripted downloads, so this reads the unofficial
Hugging Face mirror GlowBond/EuRoC_MAV_Dataset. Its machine_hall.zip is 12.7 GB; HTTP
range reads pull out only the three inner ASL zips (4 GB), CRC-checked by zipfile.
Needs: pip install remotezip==0.12.3
"""
import os
import subprocess
import zipfile

from remotezip import RemoteZip

URL = "https://huggingface.co/datasets/GlowBond/EuRoC_MAV_Dataset/resolve/main/machine_hall.zip"
OUT = os.path.expanduser("~/data/euroc")

for seq in ["MH_01_easy", "MH_02_easy", "MH_03_medium"]:
    dest = os.path.join(OUT, seq)
    if os.path.isdir(os.path.join(dest, "mav0")):
        print(seq, "already present", flush=True)
        continue
    print("fetching", seq, flush=True)
    with RemoteZip(URL) as z:
        inner = z.extract(f"machine_hall/{seq}/{seq}.zip", path=OUT)
    os.makedirs(dest, exist_ok=True)
    with zipfile.ZipFile(inner) as iz:
        bad = iz.testzip()
        assert bad is None, f"corrupt member {bad}"
        iz.extractall(dest)
    os.remove(inner)
    print(seq, "done:", subprocess.run(["du", "-sh", dest], capture_output=True, text=True).stdout.strip(), flush=True)
print("FETCH_DONE", flush=True)
