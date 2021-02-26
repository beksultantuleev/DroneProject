import paho.mqtt.client as mqtt
import re
import numpy as np
import numpy.matlib
import array as arr
import json
import paho.mqtt.client as mqttClient
import time
import collections
import atexit

Connected = False  # global variable for the state of the connection
DEBUG = False
num_anch = 7
num_tag = 5
buffer_size = 100
slot = np.zeros((num_tag, buffer_size, num_anch),
                dtype=int)  # creo 100 slot toa
tmp = []
DBdata = {}
DBdata['measurement'] = 'TOA_Ranging'
DBdata['tags'] = {}
DBdata['tags'] = {'room': 'lab', 'anchors': 0}
DBdata['fields'] = {}
DBdata['tags']['anchors'] = 6
dist = {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0, 'A5': 0, 'A6': 0}
dict_keys = list(dist.keys())
broker_address = "127.0.0.1"  # Broker address
port_id = 1883  # Broker port
subscriptions_qos = [("tags/#", 0)]


def on_connect(client, userdata, flags, rc):
    print("Successfully connected to MQTT with result code %s" % str(rc))
    print("before message_callback_add 1")
    client.message_callback_add("tags/#", ToA_callback)
    print("after message_callback_add")

    (result, _) = client.subscribe(subscriptions_qos)
    if (result == mqtt.MQTT_ERR_SUCCESS):
        print("Successfully subscribed to MQTT topics with result code %s" %
              str(result))


def on_message(client, userdata, message):
    print("Received: Topic: %s Body: %s", message.topic, message.payload)


def ToA_callback(client, userdata, message):
    try:

        data = message.payload.split()

        if (RepresentsInt(data[2])):
            anch_id = int(data[0])
            tag_id = int(data[1])
            num_msg = int(data[2])
            toa = int(data[3])

            if DEBUG:
                print(str(message.payload.split()))
                print("num_msg: ", num_msg)
                print("anch_id ", anch_id)
                print("tag_id", tag_id)
                print("toa ", toa)

            slot[tag_id, num_msg % 100, anch_id] = toa
            for i in range(1, 5, 1):
                index = np.nonzero(
                    np.all(np.array(slot[i, :, 1:7]) != 0, axis=1))[0]
                # print(slot[i,:,1:7])
                # print(np.nonzero(np.all(slot[i,:,1:7] != 0, axis=0))[0])
                if index.size > 0:
                    slot[i, index, 0] = num_msg
                    client.publish(
                        'TOA' + str(i), str(slot[i, index, 1:7]), qos=0)

                    for h in range(1, len(dist), 1):
                        dist[dict_keys[h-1]] = int(slot[i, index, h])

                    DBdata['fields'] = dist
                    DBdata['fields']['tagID'] = tag_id
                #   print(DBdata)

                    if DEBUG:
                        print('Insert DataBase Data:')
                        print(DBdata['fields']['A1'])

                    slot[i, index, :] = [0] * (num_anch)
                    index = []
                #   print('Done')

    except:
        raise


def Localisation(self, data):
    self.data = data
    M = 6
    c = 299792458

    A_n1 = np.array([[0.00], [7.19], [2.15]])
    A_n2 = np.array([[0.00], [3.62], [3.15]])
    A_n3 = np.array([[0.00], [0.00], [2.15]])
    A_n4 = np.array([[4.79], [1.85], [3.15]])
    A_n5 = np.array([[4.79], [5.45], [2.15]])
    A_n6 = np.array([[3.00], [9.35], [3.15]])
    A_n = np.array([A_n1, A_n2, A_n3, A_n4, A_n5, A_n6])

    t1 = float(self.data[0]) * (15.65e-12)
    t2 = float(self.data[1]) * (15.65e-12)
    t3 = float(self.data[2]) * (15.65e-12)
    t4 = float(self.data[3]) * (15.65e-12)
    t5 = float(self.data[4]) * (15.65e-12)
    t6 = float(self.data[5]) * (15.65e-12)

    # tolgo dall'inizio ancora riferimento e lascio il valore fuori (tm)
    toa = np.array([[t3, t1, t2, t4, t5, t6]])
    tdoa = toa[0][0] - toa
    tdoa = tdoa[0][1:]
    D = tdoa*c
    # A = 2*[A_n3[0][0]-A_n[:,0], A_n3[1][0]-A_n[:,1], A_n3[2][0]-A_n[:,2], D]
    # b = D**2 +  pow(np.linalg.norm(A_n3),2) -np.sum(pow(A_n[:,:],2),1)
    # x_t0 = np.linalg.pinv(A)*b
    # ______________________modified script
    A_diff_one = np.array((A_n3[0][0]-A_n[:, 0]).T[0][1:], dtype='float')
    A_diff_two = np.array((A_n3[1][0]-A_n[:, 1]).T[0][1:], dtype='float')
    A_diff_three = np.array((A_n3[2][0]-A_n[:, 2]).T[0][1:], dtype='float')

    A = 2 * np.array([A_diff_one, A_diff_two, A_diff_three, D])
    b = D**2 + np.linalg.norm(A_n3)**2 - np.sum(A_n[:, :]**2, 1)  #
    x_t0 = np.dot(b, np.linalg.pinv(A))  # b*A or A*b??

    x_t0 = np.array([x_t0[1], x_t0[2], x_t0[3]])


    f = np.zeros((n-1, 1))
    del_f = np.zeros((n-1, 3))

    for ii in range(2, n):
        f[ii-1] = np.linalg.norm(x_t0-A_n[:, ii])-np.linalg.norm(x_t0-A_n[:, 1])
        del_f[ii-1, 1] = (x_t0[1]-A_n[1, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                          ) - (x_t0[1]-A_n[1, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))
        del_f[ii-1, 2] = (x_t0[2]-A_n[2, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                          ) - (x_t0[2]-A_n[2, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))
        del_f[ii-1, 3] = (x_t0[3]-A_n[3, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                          ) - (x_t0[3]-A_n[3, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))


    x_t = np.linalg.pinv(del_f)*(D-f) + x_t0


# ______________________
# ---------- fit sol x_lin ---------

p_t_0 = [[x_lin[0]], [x_lin[1]], [x_lin[2]]]

 d = [0]*(M-1)
  cont = 0
   while cont < len(tdoa):
        d[cont] = [c*tdoa[cont][0]]
        cont += 1

    f = [0]*(M-1)
    del_f = [[0]*(M-1), [0]*(M-1), [0]*(M-1)]

    ii = 1
    while ii < M:
        f[ii-1] = [np.linalg.norm(np.subtract(p_t_0, P[ii][:])) -
                   np.linalg.norm(np.subtract(p_t_0, P[0][:]))]

        del_f[0][ii-1] = (np.subtract(p_t_0[0], P[ii][0]) * (np.linalg.norm(np.subtract(p_t_0, P[ii][:])) ** (-1))
                          ) - (np.subtract(p_t_0[0], P[0][0]) * (np.linalg.norm(np.subtract(p_t_0, P[0][:])) ** (-1)))

        del_f[1][ii-1] = (np.subtract(p_t_0[1], P[ii][1]) * (np.linalg.norm(np.subtract(p_t_0, P[ii][:])) ** (-1))
                          ) - (np.subtract(p_t_0[1], P[0][1]) * (np.linalg.norm(np.subtract(p_t_0, P[0][:])) ** (-1)))

        del_f[2][ii-1] = (np.subtract(p_t_0[2], P[ii][2]) * (np.linalg.norm(np.subtract(p_t_0, P[ii][:])) ** (-1))
                          ) - (np.subtract(p_t_0[2], P[0][2]) * (np.linalg.norm(np.subtract(p_t_0, P[0][:])) ** (-1)))

        ii += 1

    pinv_del_f = np.linalg.pinv(np.transpose(del_f))
    c_diff = np.subtract(d, f)

    pos = [0] * 3

    pos = np.dot(pinv_del_f, c_diff) + p_t_0

    position = [pos[0][0][0], pos[0][1][0], pos[0][2][0]]


def RepresentsInt(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


def main():
    # logger = logging.getLogger('root')
    # logging.basicConfig(format='[%(asctime)s %(levelname)s: %(funcName)20s] %(message)s', level=logging.DEBUG)

    client = mqtt.Client()
    # client.on_log = on_log
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address, port_id)
    client.loop_forever()


if __name__ == '__main__':
    main()
