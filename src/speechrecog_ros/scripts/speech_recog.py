import rospy
from std_msgs.msg import String
from speechrecog_ros.msg import SaladRecipe, SaladCommand 
import sys
import os
import pyaudio
import speech_recognition as sr
import json
import re
import openai
import signal
#import pyttsx3
from gtts import gTTS
from playsound import playsound



def signal_handler(sig, frame):
    rospy.loginfo("Ctr+C captured, exiting gracefully...")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def recognize_speech(recognizer, microphone):
    with microphone as source:
        rospy.loginfo("Adjusting for ambient noise. Please wait.")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        rospy.loginfo("Listening...")
        try:
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
        except sr.WaitTimeoutError:
            rospy.loginfo("No speech detected within the timeout period.")
            return None
    try:
        rospy.loginfo("Processing audio...")
        recognized_text = recognizer.recognize_google(audio)
        rospy.loginfo(f"You said: {recognized_text}")
        return recognized_text
    except sr.UnknownValueError:
        rospy.loginfo("Sorry, I could not understand the audio. Please try again.")
    except sr.RequestError as e:
        rospy.logerr(f"Could not request results from the speech recognition service; {e}")
    return None


variables list to send thru openai

def send_to_openai(prompt):
    OPENAI_API_KEY =''
    if not OPENAI_API_KEY:
        rospy.logerr("OpenAI API key not found. Please set the OPENAI_API_KEY environment variable.")
        sys.exit(1)
    openai.api_key = OPENAI_API_KEY

    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": (
                    "You are a helpful assistant that provides salad recipes and instructions. "
                    "When providing recipes, respond in the following JSON format:\n\n"
                    "{\n"
                    "  \"commands\": [\n"
                    "    {\n"
                    "      \"motion\": \"string\",\n"
                    "      \"object\": \"string\",\n"
                    "      \"optional_target\": \"string\"\n"
                    "    },\n"
                    "    ...\n"
                    "  ]\n"
                    "}\n"
                    "Only respond with JSON when appropriate."
                )},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7
        )
        assistant_response = response.choices[0].message.content.strip()
        rospy.loginfo(f"Assistant Response: {assistant_response}")
        return assistant_response
    except Exception as e:
        rospy.logerr(f"Error communicating with OpenAI: {e}")
        return None

def parse_salad_recipe(response_text):
    try:
        json_pattern = r'\{.*\}'
        match = re.search(json_pattern, response_text, re.DOTALL)
        if not match:
            rospy.loginfo("No JSON found in the response.")
            return None

        json_text = match.group(0)
        data = json.loads(json_text)

        #this the saladrecipe message
        recipe = SaladRecipe()
        for cmd in data.get('commands', []):
            command = SaladCommand()
            command.motion = cmd.get('motion', '')
            command.object = cmd.get('object', '')
            command.optional_target = cmd.get('optional_target', '')
            recipe.commands.append(command)
        return recipe
    except json.JSONDecodeError as e:
        rospy.logerr(f"JSON decoding error: {e}")
    except Exception as e:
        rospy.logerr(f"Error parsing SaladRecipe: {e}")
    return None

def speak_text(text):
    try:
        tts = gTTS(text=text, lang = 'en')
        filename = "temp_audio.mp3"
        tts.save(filename)
        playsound(filename)
        os.remove(filename)
    except Exception as e:
        rospy.logerr(f"Error in tts: {e}")

def main():
    rospy.init_node('voice_assistant_node')
    speech_pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    response_pub = rospy.Publisher('assistant_response', String, queue_size=10)
    recipe_pub = rospy.Publisher('salad_recipe', SaladRecipe, queue_size=10)

    recognizer = sr.Recognizer()
    DEVICE_INDEX = 2  
    microphone = sr.Microphone(device_index=DEVICE_INDEX)

    #engine = pyttsx3.init()
    #engine.setProperty('rate', 150)
    #engine.setProperty('volume', 1.0)

    rospy.loginfo("Welcome to Baxter the Salad Assistant!")
    rospy.loginfo("You can ask me anything about making salads.")
    rospy.loginfo("Say 'exit', 'quit', or 'stop' to end the conversation.")

    while not rospy.is_shutdown():
        voice_input = recognize_speech(recognizer, microphone)
        if voice_input is None:
            continue  

        if voice_input.lower() in ["exit", "quit", "stop"]:
            rospy.loginfo("Goodbye!")
            rospy.signal_shutdown("User requested exit.")
            break

        speech_pub.publish(voice_input)

        assistant_response = send_to_openai(voice_input)
        if assistant_response is None:
            continue

        response_pub.publish(assistant_response)

        speak_text(assistant_response)

        salad_recipe = parse_salad_recipe(assistant_response)
        if salad_recipe:
            rospy.loginfo("Publishing SaladRecipe to 'salad_recipe' topic.")
            recipe_pub.publish(salad_recipe)
        else:
            rospy.loginfo("No SaladRecipe found in the response.")

if __name__ == "__main__":
    main()
