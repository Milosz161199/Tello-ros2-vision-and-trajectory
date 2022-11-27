# ARL_PROJ_GRUPA_IV
Wykrywanie kartki

Aby uruchomić serwer akcji odpowiadajacej za wykrywanie kartki uruchowmic w teminale skrypt przy pomocy:
`python3 src/paper_detection/paper_detection/detect_server.py`

Następnie w osobnym terminalu:
`ros2 action send_goal /Detect action_detect/action/Detect order:\ 0\`
