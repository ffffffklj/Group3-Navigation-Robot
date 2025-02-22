#!/usr/bin/env python3
import openai
import rospy
import speech_recognition as sr
from std_msgs.msg import String

openai.api_key = "TODO"

def recognize_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        rospy.loginfo("Listening for voice command...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_google(audio)
        rospy.loginfo(f"Recognized: {text}")
        return text
    except sr.UnknownValueError:
        rospy.logwarn("Could not understand audio")
        return ""
    except sr.RequestError:
        rospy.logwarn("Could not request results, check internet connection")
        return ""

def query_openai(text):
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": text}]
    )
    return response["choices"][0]["message"]["content"]

def voice_command_node():
    rospy.init_node("voice_command_node")
    pub = rospy.Publisher("/navibot/voice_command", String, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        command = recognize_speech()
        if command:
            response = query_openai(command)
            rospy.loginfo(f"AI Response: {response}")
            pub.publish(response)
        rate.sleep()

if __name__ == "__main__":
    voice_command_node()
