import opcua
from opcua import Client

# OPC UA servers (igual seu código)
Leitstand     = "opc.tcp://192.168.97.101:4840"
Palettenlager = "opc.tcp://192.168.97.102:4840"
Handling      = "opc.tcp://192.168.97.103:4840"
Presse_Press  = "opc.tcp://192.168.97.104:4840"

user = "MES"
password = "training"

def connect_opcua():

    global client_ControlStation
    global root_ControlStation

    try:

        print("Connecting to Leitstand...")

        client_ControlStation = Client(Leitstand)
        client_ControlStation.set_user(user)
        client_ControlStation.set_password(password)

        client_ControlStation.connect()

        root_ControlStation = client_ControlStation.get_root_node()

        print("Connected successfully!")

        return True

    except Exception as e:

        print("Connection error:", e)
        return False


def test_read_value():

    try:

        print("Reading order status...")

        order_status = root_ControlStation.get_children()[0] \
                        .get_children()[4] \
                        .get_children()[8] \
                        .get_value()

        print("Order Status =", order_status)

    except Exception as e:

        print("Read error:", e)


def disconnect():

    try:
        client_ControlStation.disconnect()
        print("Disconnected")

    except Exception as e:
        print("Disconnect error:", e)


if __name__ == "__main__":

    if connect_opcua():

        test_read_value()

        disconnect()