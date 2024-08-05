# pull docker
```bash
docker pull hazardyyp/data_recording:image_classifier
```
# or build from dockerfile
```bash
docker build -t data_recording:image_classifier -f Dockerfile .
```

# to run
```bash
mkdir 20240806 #rosbag save dir
cd data_recording
bash start_recording.sh
```

# common issue
## display issue
```bash
xhost +local:docker
```
