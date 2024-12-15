import rospy
from std_msgs.msg import String
import vosk
import pyaudio
import pyttsx3
import json
from geometry_msgs.msg import Twist  # Considering that this form of message is used to control robot movements

# Get the text-to-speech speech engine started.
engine = pyttsx3.init()

# function to regulate robot motions (e.g., nod, wave)
def perform_gesture(command):
    gesture_pub = rospy.Publisher('/robot/gesture', Twist, queue_size=10)
    twist = Twist()

    if 'wave' in command:
        # Configure the waving gesture's parameters.
        twist.linear.x = 1.0  # Adapt to the robot's waving API.
        rospy.loginfo("Waving")
    elif 'nod' in command:
        # Configure the nodding gesture's parameters.
        twist.angular.z = 1.0  # Adapt for nodding based on the robot's API
        rospy.loginfo("Nodding")
    elif 'shake' in command:
        # Configure the shaking gesture's parameters.
        twist.angular.z = -1.0  # Adjust for shaking head
        rospy.loginfo("Shaking head")
    else:
        twist.linear.x = 0
        twist.angular.z = 0
    
    # Publish gesture
    gesture_pub.publish(twist)

# The ability to recognize and react to speech
def listen_and_respond():
    # Initialize ROS node
    rospy.init_node('speech_listener_and_responder', anonymous=True)
    
    # Set up PyAudio and the Vosk model initially.
    model = vosk.Model("model")
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
    stream.start_stream()
    
    # Build a speech recognition system
    recognizer = vosk.KaldiRecognizer(model, 16000)
    
    # Pay attention to the microphone and identify speech.
    while not rospy.is_shutdown():
        data = stream.read(4000)
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            result_dict = json.loads(result)
            speech_text = result_dict.get("text", "").lower()
            rospy.loginfo(f"Recognized speech: {speech_text}")

            # Verify that your speech fits the predetermined phrases, then adjust your response accordingly.
            if 'hi' in speech_text or 'hello' in speech_text:
                engine.say("Hello! How can I assist you?")
                engine.runAndWait()
                perform_gesture("wave")  # Robot waves
            elif 'yes' in speech_text:
                engine.say("Yes, I am listening.")
                engine.runAndWait()
                perform_gesture("nod")  # Robot nods
            elif 'no' in speech_text:
                engine.say("No, I am not sure about that.")
                engine.runAndWait()
                perform_gesture("shake")  # Robot shakes head
            else:
                engine.say(f"You said: {speech_text}")
                engine.runAndWait()

# Publication function for text-to-speech
def speak_string():
    rospy.init_node('speech_publisher', anonymous=True)
    pub = rospy.Publisher('/tts/phrase', String, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        user_input = input("Enter a phrase for TTS: ")
        pub.publish(user_input)
        rate.sleep()

if __name__ == '__main__':
    try:
        # You have the option of running the speaking or listening node.
        # listen_and_respond() # For gestural listening and response
        speak_string()  # To publish commands that are text-to-speech
    except rospy.ROSInterruptException:
        pass
