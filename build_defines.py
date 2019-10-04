import subprocess
import datetime

gitrevision = subprocess.check_output(["git", "rev-parse", "HEAD"]).strip()

currentDT = datetime.datetime.now()
builddate = currentDT.strftime("%Y-%m-%d")
buildtime = currentDT.strftime("%H:%M:%S")


print("-DPIO_SRC_REV=%s -DPIO_BUILD_DATE=%s -DPIO_BUILD_TIME=%s" %(gitrevision.decode("utf-8"),builddate,buildtime)),

