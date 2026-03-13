from flask import Flask, render_template, request, url_for, jsonify, Response
from flask import jsonify
import threading
import opcua
from opcua import Client
import logging
import datetime
import configparser
import pandas as pd
import time
import random

app = Flask(__name__)

# Define your OPC UA connection settings here
Leitstand     = "opc.tcp://192.168.97.101:4840"
Palettenlager = "opc.tcp://192.168.97.102:4840"
Handling      = "opc.tcp://192.168.97.103:4840"
Presse_Press  = "opc.tcp://192.168.97.104:4840"

user = "MES"
password = "training"

root_ControlStation = None
root_PalletStore    = None
root_Handling       = None
root_Press          = None

# Define a timer start time
start_time = time.time()

# Define a maximum time limit (30 seconds)
max_time_limit = 30



@app.route('/')
def index():
    return render_template('index.html')

@app.route('/connect', methods=['POST'])
def connect_opcua():
    # Declaring global root and client variables for control station
    global client_ControlStation
    global root_ControlStation
    # Declaring global root and client variables for Pallet Store
    global client_PalletStore
    global root_PalletStore
    # Declaring global root and client variables for Handling
    global client_Handling
    global root_Handling
    # Declaring global root and client variables for Press
    global client_Press
    global root_Press
    if request.method == 'POST':
        try:
            # Create an instance of the OPC UA Client and connecting it
            client_ControlStation = Client(Leitstand)
            client_ControlStation.set_user(user)
            client_ControlStation.set_password(password)
            client_ControlStation.connect()
            root_ControlStation = client_ControlStation.get_root_node()
            status1= '1. Connected to OPC UA Leitstand server successfully.'

            client_PalletStore = Client(Palettenlager)
            client_PalletStore.set_user(user)
            client_PalletStore.set_password(password)
            client_PalletStore.connect()
            root_PalletStore = client_PalletStore.get_root_node()
            status2= '2. Connected to OPC UA Palettenlager server successfully.'

            client_Handling = Client(Handling)
            client_Handling.set_user(user)
            client_Handling.set_password(password)
            client_Handling.connect()
            root_Handling = client_Handling.get_root_node()
            status3= '3. Connected to OPC UA Handling server successfully.'

            client_Press = Client(Presse_Press)
            client_Press.set_user(user)
            client_Press.set_password(password)
            client_Press.connect()
            root_Press = client_Press.get_root_node()
            status4= '4. Connected to OPC UA Press server successfully.'
            status = status1 + '<br>' +  status2 + '<br>' + status3 + '<br>' + status4
        except Exception as e:
            status1 = f'Error connecting to OPC UA Leitstand server: {str(e)}'
            status2 = f'Error connecting to OPC UA Palettenlager server: {str(e)}'
            status3 = f'Error connecting to OPC UA Handling server: {str(e)}'
            status4 = f'Error connecting to OPC UA Press server: {str(e)}'
            status = status1 + '<br>' +  status2 + '<br>' + status3 + '<br>' + status4
         # Pass both status and root_value to the template
        return render_template('index.html', status=status)

@app.route('/disconnect', methods=['POST'])
def disconnect_opcua():
    if request.method == 'POST':
        try:
            client_ControlStation.disconnect()
            status1 = '1. Disconnected from OPC UA Leitstand server.'
            client_PalletStore.disconnect()
            status2 = '2. Disconnected from OPC UA Palettenlager server.'
            client_Handling.disconnect()
            status3 = '3. Disconnected from OPC UA Handling server.'
            client_Press.disconnect()
            status4 = '4. Disconnected from OPC UA Press server.'
            status = status1 + '<br>' +  status2 + '<br>' + status3 + '<br>' + status4
            root_ControlStation = None
            root_PalletStore    = None
            root_Handling       = None
            root_Press          = None
        except Exception as e:
            status1= f'Error disconnecting from OPC UA Leitstand server: {str(e)}'
            status2 = f'Error disconnecting from OPC UA Palettenlager server: {str(e)}'
            status3 = f'Error disconnecting from OPC UA Handling server: {str(e)}'
            status4 = f'Error disconnecting from OPC UA Press server: {str(e)}'
            status = status1 + '<br>' +  status2 + '<br>' + status3 + '<br>' + status4
        return render_template('index.html', status=status)

@app.route('/set_color', methods=['POST'])
def set_color():
    global selected_color
    if request.method == 'POST':
        try:
            # Check if the OPC UA client (root) is connected
            if root_ControlStation is None:
                status_color = 'Error: OPC UA client is not connected. Connect first.'
            else:
                # Get the selected color from the form
                selected_color = int(request.form['color'])
                
                # Create a DataValue with the selected color
                dv_color = opcua.ua.DataValue(opcua.ua.Variant(selected_color, opcua.ua.VariantType.Byte))
                dv_color.ServerTimestamp = None
                dv_color.SourceTimestamp = None
                
                # Set the color value in OPC UA
                root_ControlStation.get_children()[0].get_children()[4].get_children()[1].set_value(dv_color)

                # Reading the updated value of the color
                updated_color = root_ControlStation.get_children()[0].get_children()[4].get_children()[1].get_value()
                
                # Check if the color was set correctly
                if updated_color == selected_color:
                    status_color = f'Color set to {selected_color} ({"Red" if selected_color == 1 else "Blue"}) successfully.'
                else:
                    status_color = f'Error setting color. Updated color: {updated_color}'
        except Exception as e:
            status_color = f'Error setting color: {str(e)}'
        return render_template('index.html', status_color=status_color)
        
@app.route('/set_color_status', methods=['GET'])
def set_color_status():
    try:
        selected_color_status = selected_color
        if selected_color_status == 1:
            return jsonify({'selected_color_status': 'Red'})
        elif selected_color_status == 4:
            return jsonify({'selected_color_status': 'Blue'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

@app.route('/get_order_status', methods=['GET'])
def get_order_status():
    try:
        # Read the order status from the OPC UA server
        order_status = root_ControlStation.get_children()[0].get_children()[4].get_children()[8].get_value()
        return jsonify({'order_status': order_status})
    except Exception as e:
        return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

@app.route('/set_quantity', methods=['POST'])
def set_quantity():
    global selected_quantity
    if request.method == 'POST':
        try:
            # Check if the OPC UA client (root) is connected
            if root_ControlStation is None:
                status_quantity = 'Error: OPC UA client is not connected. Connect first.'
            else:
                # Get the selected quantity from the form
                selected_quantity = int(request.form['quantity'])
                status_quantity = f'Quantity set to {selected_quantity}'
        except Exception as e:
            status_quantity = f'Error setting color: {str(e)}'
        return render_template('index.html', status_quantity=status_quantity)
 

@app.route('/set_quantity_status', methods=['GET'])
def set_quantity_status():
    try:
        selected_quantity_status = selected_quantity
        return jsonify({'selected_quantity_status': selected_quantity_status})
    except Exception as e:
        return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

@app.route('/get_OrderNumber_status', methods=['GET'])
def get_OrderNumber_status():
    try:
        order_number_status = Order_Number
        return jsonify({'order_number_status': order_number_status})
    except Exception as e:
        return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

@app.route('/process_order', methods=['POST'])
def process_order():
    global Assembly_Status
    global Order_Number
    Order_Number = random.randint(1000,9999)
    # Setting the order number 
    dv_order = opcua.ua.DataValue(opcua.ua.Variant(Order_Number, opcua.ua.VariantType.Int16))
    dv_order.ServerTimestamp = None
    dv_order.SourceTimestamp = None
    # Setting the order value
    root_ControlStation.get_children()[0].get_children()[4].get_children()[0].set_value(dv_order)
    Assembly_Status = f"Assembly process started for the Order Number:  {Order_Number}."
    time.sleep(5)
    if request.method == 'POST':
        try:      
            for quantity in range(1,selected_quantity+1):
                order_status_check = root_ControlStation.get_children()[0].get_children()[4].get_children()[8].get_value()
                if order_status_check:
                    Pallet_Storage_1 = root_PalletStore.get_children()[0].get_children()[4].get_children()[5].get_value()
                    Pallet_Storage_2 = root_PalletStore.get_children()[0].get_children()[4].get_children()[6].get_value()
                    # Check if the Pallet storage has 10 pallets or 0 pallet
                    if Pallet_Storage_1 == 10:
                        return jsonify({'message':'Order cannot be placed. The Pallets in the pallet storage must be less than 10'})
                    elif Pallet_Storage_1 == 0:
                        return jsonify({'message':'Order cannot be placed. Add Pallets to the pallet storage'})
                    elif Pallet_Storage_2 == 12:
                        return jsonify({'message': 'Order cannot be placed. Empty the Pallets from pallet storage 2'}) 
                    # Give the order if the status is true
                    give_order_true = opcua.ua.DataValue(opcua.ua.Variant(True, opcua.ua.VariantType.Boolean))
                    give_order_true.ServerTimestamp = None
                    give_order_true.SourceTimestamp = None
                    root_ControlStation.get_children()[0].get_children()[4].get_children()[6].set_value(give_order_true)                
                    while root_PalletStore.get_children()[0].get_children()[4].get_children()[3].get_value() != 2:
                        pass
                    Assembly_Status = f"Process has started for Box Number {quantity}."

                    while root_ControlStation.get_children()[0].get_children()[4].get_children()[7].get_value() != True:
                        pass
                    give_order_false = opcua.ua.DataValue(opcua.ua.Variant(False, opcua.ua.VariantType.Boolean))
                    give_order_false.ServerTimestamp = None
                    give_order_false.SourceTimestamp = None
                    root_ControlStation.get_children()[0].get_children()[4].get_children()[6].set_value(give_order_false) 

                    while root_PalletStore.get_children()[0].get_children()[4].get_children()[3].get_value() != 4:
                        pass
                    Assembly_Status = "Palet is successfully moved to the handling process"

                    while  root_Handling.get_children()[0].get_children()[4].get_children()[3].get_value() == 0:
                        pass
                    Assembly_Status = "Handling process has started" 

                    while  root_Handling.get_children()[0].get_children()[4].get_children()[3].get_value() != 0:
                        pass
                    Assembly_Status = "Handling process has successfully completed" 
                   
                    while  root_Press.get_children()[0].get_children()[4].get_children()[3].get_value() == 0:
                        pass
                    Assembly_Status = "Lid pressing process has started" 

                    while root_Press.get_children()[0].get_children()[4].get_children()[3].get_value() != 4:
                        pass
                    Assembly_Status = "Lid pressing process has successfully completed" 
                            
                    while root_PalletStore.get_children()[0].get_children()[4].get_children()[3].get_value() != 2:
                        pass
                    Assembly_Status = "The assembled box is successfully stored in the pallet storage" 
                    time.sleep(5) # Waiting for the Palet storage to update its values
                    if (root_PalletStore.get_children()[0].get_children()[4].get_children()[6].get_value() == (Pallet_Storage_2 + 1)) and (root_PalletStore.get_children()[0].get_children()[4].get_children()[5].get_value() == (Pallet_Storage_1 - 1)): 
                        Assembly_Status = f"Box Number {quantity} is successfully assembled"
                        if(quantity == selected_quantity):
                            Assembly_Status = 'No order for processing'
                            return jsonify({'message': f"The Order number {Order_Number} is successfully completed"})                   
                    else:
                        Assembly_Status = "Assembly process not completed successfully" 
                        Assembly_Status = 'No order for processing'
                        return jsonify({'message': f"The Order number {Order_Number} cannot be completed due to some error"})  
                    # Giving delay to the next order to start    
                    time.sleep(5)                 
                else:
                    return jsonify({'message': 'Order status is not true. Cannot give an order.'})
        except Exception as e:
            return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

@app.route('/get_assembly_step', methods=['GET'])
def get_assembly_step():
    try:
        # Read the assembly step from the global variable defined in the process order function
        return jsonify({'Assembly_Status': Assembly_Status})
    except Exception as e:
        return jsonify({'error': str(e)}), 500  # Handle errors gracefully and return a 500 status code

if __name__ == '__main__':
    app.run(debug=True)
