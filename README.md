This project serves as a 3-tier network (sensors, edge, cloud) to detect motion from one side of a doorway to the other, and verify that things passing through the doorway are humans using some lightweight Computer Vision (CV), using the AI model YOLO (You Only Look Once).
This project was originally created to run across a Raspberry Pi 4, AWS VM and Teensy 4.0/2.0 hardware, however the python code for the edge and cloud tier are hardware agnostic. The edge tier has been modified from its original to utilise a default webcam dictated by the OS, where previously it used a raw serial feed.
If adding AI functionality or changing the model used to detect people, please refer to the internal comments.
AI model is defined by the variable "model"
The logic around the implementation of the AI model is defined within the function "validate_person"
