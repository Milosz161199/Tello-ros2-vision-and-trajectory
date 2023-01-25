# ARL_PROJ_GRUPA_IV
## Temat: Odczytywanie i odtwarzanie przez dron trajektorii narysowanej na kartce.

Autorzy:
- Albert Polak
- Krzysztof Kwapisz
- Miłosz Plutowski

Założenia projektu:
- dron odczytuje obraz trajektorii z kamery,
- przetwarza trajektorię (uwzględniając ruch w trzech osiach x, y, z): x, y - kartka, z - kolor ścieżki,
- porządkuje punkty trajektorii,
- wykonuje trajektorię i wraca do punktu bazowego, i ląduje.

Projekt można uruchomić w symulacji, jak i na rzeczywistym dronie. Na potrzeby projektu zostały również przygotowane węzły repubulikujące wiadomości zawierające pozycję z OptiTrack ([UDP_to_ROS2_OptiTrack_republisher](https://github.com/Kwach00/UDP_to_ROS2_OptiTrack_republisher.git)) i g2rr. 

Wszystkie założenia udało się zrealizować. Dodatkowo przygotowaliśmy możliwość podglądu w czasie rzeczywistym punktów do odwiedzenia oraz bierzącej pozycji drona. Dzięki czemu możemy przewidywać jaki ruch wykona dron. 

**Wersja obrazu: osrf/ros:foxy**

Potrzebne biblioteki:
```
numpy==1.23.4
opencv-contrib-python==4.5.5.64
opencv-python==4.5.5.64
transforms3d==0.4.1
protobuf==3.20.0
tqdm==4.64.1
matplotlib==3.1.2
imutils==0.5.4
```

<p align="center"> 
   NAJWAŻNIEJSZE ZAŁOŻENIE, KTÓRE UDAŁO SIĘ SPEŁNIĆ
</p>

![drony](https://user-images.githubusercontent.com/77626602/214558216-03d60918-4d0e-46a7-ac59-dbc05948c30f.png)

## Przykład z symulacji Gazebo
![drony_gazebo_01](https://user-images.githubusercontent.com/77626602/214559370-6e7910be-c41e-4d11-969b-da6428fb2028.png)

## Przykład z rzeczywistego drona
![drony_real](https://user-images.githubusercontent.com/77626602/214559636-ab05cac2-67c4-48dd-8fb8-38684056e8f3.jpg)


## Przygotowanie contenera
Należy pobrać obraz dockera **foxy**. Następnie podczas tworzenia konteneru należy dodać opcję (dzięki temu kontener i host ma ten sam adres IP):
```
--network=host
```
Następnie startujemy kontener i wchodzimy do kontenera za pomocą komendy:
```
docker exec -it container_name bash
```
# Wszystkie poniższe rzeczy robimy wewnątrz kontenera.
## Potrzebne rzeczy do zainstalowania w kontenerze
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

### Republisher Optitrack 
Link do repozytorium: [Kwach00/UDP_to_ROS2_OptiTrack_republisher](https://github.com/Kwach00/UDP_to_ROS2_OptiTrack_republisher.git)
```
cd ~/tello_ros_ws/src
git clone https://github.com/Kwach00/UDP_to_ROS2_OptiTrack_republisher.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### Paczka zawierająca symulację świata z dronem w gazebo
Do uruchomienia symulacji konieczne będzie pobranie poniższej paczki.
Link do repozytorium: [TIERS/tello-ros2-gazebo](https://github.com/TIERS/tello-ros2-gazebo.git)
```
cd ~/tello_ros_ws/src
git clone https://github.com/TIERS/tello-ros2-gazebo.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

### Podmienienie tekstur w arucotag'ach
Należy przekopiować zdjęcie z folderu **src/path_models/** do katalogu **src/tello-ros2-gazebo/tello_ros/tello_gazebo/models/marker_0/materials/textures/**, 
następnie w pliku **src/tello-ros2-gazebo/tello_ros/tello_gazebo/models/marker_0/materials/scripts/marker_0.material** podmienić nazwę *.png na nazwę skopiowanego wcześniej zdjęcia.

# Uruchomienie (dwa tryby):
## **Symulacja** musimy w kodzie zmienić odpowiednie flagi (contorl_client.py oraz detect_server.py)
```
# contorl_client.py
self.operate_in_sim = True
# detect_server.py
self.__simulation = True
```
Następnie w osobnych terminalach uruchamiamy następujące polecenia:

**W każdy nowym otwartym termianlu należy przejść do głownego katalogo i wykonać source!**
```
cd tello_ros_ws/
source install/setup.bash
```

Uruchamiamy symulację świata wraz z dronem w gazebo:
```
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
ros2 launch tello_gazebo simple_launch.py
```

Republishera topic'ów z gazebo
```
ros2 run pkg_g2rr g2rr tello_1
```

Serwer z detekcją kartki i generacją ścieżki: 
```
python3 src/paper_detection/paper_detection/detect_server.py
```

Klient odpowiadającego za sterowanie dronem:
```
python3 src/drone_control/drone_control/control_client.py
```

## **Rzeczywisty dron** zmieniamy flagi na przeciwne (contorl_client.py oraz detect_server.py)
```
# contorl_client.py
self.operate_in_sim = False
# detect_server.py
self.__simulation = False
```
Następnie w osobnych terminalach uruchamiamy następujące polecenia: 
Do startowania rzeczywistym drone potrzebujemy uruchomić paczkę teleop
```
ros2 launch tello_driver teleop_launch.py
```

Republisher Optitrack'a 
```
ros2 run udp_republisher udp_republisher
```

Serwer z detekcją kartki i generacją ścieżki: 
```
python3 src/paper_detection/paper_detection/detect_server.py
```

Klient odpowiadającego za sterowanie dronem:
```
python3 src/drone_control/drone_control/control_client.py
```


Do testów prykład żądania akcji:
```
ros2 action send_goal /Detect action_detect/action/Detect order:\ 0
```

## Przydatne polecenia kiedy np. coś pójdzie nie tak
Lądowanie:
```bash
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
```
Startowania:
```bash
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" 
```
Reset symulacji:
```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
```
