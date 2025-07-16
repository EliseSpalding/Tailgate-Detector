import boto3
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc): # func for making connection
	print("Connected to TailGateDetectionSystem")
	print("Connection returned result: " + str(rc) )

	client.subscribe("personDetection")

def send_sns_alert(message):
    try:
        response = sns_client.publish(
            TopicArn=SNS_TOPIC_ARN,
            Message=message,
            Subject="MQTT Alert"
        )
        print(f"SNS Alert Sent: {response}")
    except Exception as e:
        print(f"Failed to send SNS alert: {e}")

def on_message(client, userdata, message):
    alert_message = message.payload.decode('utf-8').strip('\r\n')
    print(f"Received MQTT Message: {alert_message}")
    send_sns_alert(alert_message)

sns_client = boto3.client('sns', region_name='ap-southeast-2')
#This SNS has been disabled
SNS_TOPIC_ARN = "arn:aws:sns:ap-southeast-2:324441770365:TailGate-Message-Service"
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

#This VM has been decomissioned
client.connect("ec2-54-252-150-42.ap-southeast-2.compute.amazonaws.com", 1883, 60)

client.loop_forever()
