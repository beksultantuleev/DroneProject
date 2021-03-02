from subscriber import MqttSubscriber
from ModelBasedAgent_UWB_thread import ModelBasedAgentUWB

# ======================= MQTT ================================
uav = ModelBasedAgentUWB()
uav.mambo.smart_sleep(1)
mqttSub = MqttSubscriber("192.168.1.200", topic="Position2")
uav.mambo.smart_sleep(1)
mqttSub.start()
uav.mambo.smart_sleep(1)
uav.start_and_prepare()
uav.mambo.smart_sleep(1)
uav.mambo.hover()
uav.mambo.smart_sleep(1)
uav.land_and_disconnect()