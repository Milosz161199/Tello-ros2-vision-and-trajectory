# ARL_PROJ_GRUPA_IV
Odczytywanie i odtwarzanie przez dron trajektorii narysowanej na kartce

## Potrzebne rzeczy do zainstalowania
```
pip3 install tqdm protobuf==3.20.0
sudo apt-get install python3-tk
```

## Pobieranie i tworzenie przestrzeni roboczej
Środowisko zostało przygotowane w dockerze dla ros2 foxy.
```
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/Milosz161199/ARL_PROJ_GRUPA_IV.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```
### Paczka zawierająca symulację świata z dronem w gazebo
Do uruchomienia symulacji konieczne będzie pobranie poniższej paczki.
Link do repozytorium: [TIERS/tello-ros2-gazebo](https://github.com/TIERS/tello-ros2-gazebo.git)
```
mkdir -p ~/tello_ros_ws/src
cd ~/tello_ros_ws/src
git clone https://github.com/TIERS/tello-ros2-gazebo.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```
### Podmienienie tekstur w arucotag'ach
Należy przekopiować zdjęcie z folderu **src/path_models/** do katalogu **src/tello-ros2-gazebo/tello_ros/tello_gazebo/models/marker_0/materials/textures/**, 
następnie w pliku **src/tello-ros2-gazebo/tello_ros/tello_gazebo/models/marker_0/materials/scripts/marker_0.material** podmienić nazwę *.png na nazwę skopiowanego wcześniej zdjęcia.

## Uruchomienie 
W osobnych terminalach uruchamiamy następujące polecenia: 
Zaczynamy od uruchomienia symulacji:
```
cd ~/tello_ros_ws
source install/setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
ros2 launch tello_gazebo simple_launch.py
```

Aby uruchomić serwer akcji odpowiadajacej za wykrywanie kartki uruchowmic w teminalu skrypt przy pomocy: 
```
python3 src/paper_detection/paper_detection/detect_server.py
```

Klienta odpowiadającego za sterowanie dronem:
```
python3 src/drone_control/drone_control/control_client.py
```

Uruchonienie republishera topic'ów z gazebo
```
ros2 run pkg_g2rr g2rr tello_1
```

Żądanie akcji:
```
ros2 action send_goal /Detect action_detect/action/Detect order:\ 0
```
