Import("env")
import subprocess, time

port = env.GetProjectOption("upload_port", "/dev/ttyUSB0")
subprocess.call(["fuser", "-k", port], stderr=subprocess.DEVNULL)
time.sleep(1.5)
