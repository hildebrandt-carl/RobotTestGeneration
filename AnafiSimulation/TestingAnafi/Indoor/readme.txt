launch sphinx:
sphinx /home/autosoftlab/Desktop/RobotTestGeneration/AnafiSimulation/sphinxfiles/drones/anafi4k.drone::stolen_interface=enp0:eth0:192.168.42.1/24 --datalog --datalog-rate=1

turn off gps:
echo '{"jsonrpc": "2.0", "method": "SetParam", "params": {"machine":"anafi4k", "object":"gps/gps", "parameter":"out_of_order", "value":"true"}, "id": 1}' | curl -d @- http://localhost:8383 | python -m json.tool

Start recording position of robot:
tlm-data-logger inet:127.0.0.1:9060 | grep worldPosition > simulation_output_nogps_actual.txt

run file:
source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
python3 cagetext.py