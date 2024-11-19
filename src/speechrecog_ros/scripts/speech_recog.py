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



def send_to_openai(messages):
    OPENAI_API_KEY = 
        rospy.logerr("OpenAI API key not found.")
        sys.exit(1)
    openai.api_key = OPENAI_API_KEY

    try:
        response = openai.chat.completions.create(
            model="gpt-4",
            messages=messages,
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
        json_pattern = r'<BEGIN_JSON>(.*?)<END_JSON>'
        match = re.search(json_pattern, response_text, re.DOTALL)
        if not match:
            rospy.loginfo("No JSON found in the response.")
            return None

        json_text = match.group(1).strip()
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

def speak_text(full_text):
    try:
        text_to_speak = full_text.split('<BEGIN_JSON>')[0].strip()
        if not text_to_speak:
            rospy.loginfo("No text to speak")
            return

        tts = gTTS(text=text_to_speak, lang = 'en')
        filename = "temp_audio.mp3"
        tts.save(filename)
        playsound(filename)
        os.remove(filename)
    except Exception as e:
        rospy.logerr(f"Error in tts: {e}")

def get_microphone_index(target_name):
    p = pyaudio.PyAudio()
    try:
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0 and target_name.lower() in info['name'].lower():
                rospy.loginfo(f"Found target microphone '{target_name}' at index {i}: {info['name']}")
                return i
        rospy.logerr(f"Target microphone '{target_name}' not found. Please check the connection.")
        return None
    finally:
        p.terminate()

def main():
    rospy.init_node('voice_assistant_node')
    speech_pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    response_pub = rospy.Publisher('assistant_response', String, queue_size=10)
    recipe_pub = rospy.Publisher('salad_recipe', SaladRecipe, queue_size=10)

    recognizer = sr.Recognizer()
    TARGET_MICROPHONE_NAME = "Logi Webcam C920e: USB Audio (hw:1,0)"  
    DEVICE_INDEX = get_microphone_index(TARGET_MICROPHONE_NAME)
    if DEVICE_INDEX is None:
        rospy.logerr("Exiting: Target microphone not found.")
        sys.exit(1) 
    microphone = sr.Microphone(device_index=DEVICE_INDEX)

    waiting_for_recipe_confirmation = False

    #engine = pyttsx3.init()
    #engine.setProperty('rate', 150)
    #engine.setProperty('volume', 1.0)
    use_hardcoded_ingredients = False #change to true if u wanna use

    rospy.loginfo("Welcome to Baxter the Salad Assistant!")
    rospy.loginfo("You can ask me anything about making salads.")
    rospy.loginfo("Say 'exit', 'quit', or 'stop' to end the conversation.")

    messages=[
                {"role": "system", "content": (

                    "You are Chef Baxter, a helpful cooking assistant that answers questions. The user wants to make salads. Respond naturally providing salad recipes(not in the JSON format below) and having a helpful conversation with the user. If the user asks for a recipe or advice or instructions, respond in natural language and do not output the JSON format array"
                    "Only ouput the JSON format array if the user orders or asks you to or tells you to make a salad(e.g, 'Make me a fruit salad', 'Baxter make a fruit salad', 'Show me how to make a fruit salad'). Say 'Sure' and output step by step processes as the JSON format below so it can be published, not communicated with the user."

                    "<BEGIN_JSON>\n"
                    "  {\n"
                    "    \"commands\": [\n"
                    "      {\n"
                    "        \"motion\": \"pick up\",\n"
                    "        \"object\": \"tomato\",\n"
                    "        \"optional_target\": \"\"\n"
                    "      }\n"
                    "    ]\n"
                    "  }\n"
                    "  <END_JSON>\n"
                    "- Instructions must only include the following motions:\n"
                    "  - `pick up`, `place into`, `stir`, and `mix`.\n"

                    "If the user tells you to do something with the restricted motions, also print out the JSON format."
                )},
                #{"role": "user", "content": prompt}
    ]

    while not rospy.is_shutdown():
        if use_hardcoded_ingredients:
            ingredients = ["lettuce", "tomato", "cucumber"]
            voice_input = f"I have {', '.join(ingredients)}. What can I make?"
            rospy.loginfo(f"Using hardcoded ingredients: {voice_input}")
            messages.append({"role": "user", "content": voice_input})
        else:
            voice_input = recognize_speech(recognizer, microphone)
            if voice_input is None:
                continue  

            if voice_input.lower() in ["exit", "quit", "stop"]:
                rospy.loginfo("Goodbye!")
                rospy.signal_shutdown("User requested exit.")
                break

            speech_pub.publish(voice_input)
            messages.append({"role": "user", "content":voice_input})

            
        assistant_response = send_to_openai(messages)
        if assistant_response is None:
            continue

        messages.append({"role": "user", "content": voice_input})

        response_pub.publish(assistant_response)

        # speak_text(assistant_response)

        salad_recipe = parse_salad_recipe(assistant_response)
        if salad_recipe:
            rospy.loginfo("Publishing SaladRecipe to 'salad_recipe' topic.")
            recipe_pub.publish(salad_recipe)
        else:
            rospy.loginfo("No SaladRecipe found in the response.")
        if use_hardcoded_ingredients:
            rospy.loginfo("Finished processing hardcoded ingredients")

if __name__ == "__main__":
    main()
