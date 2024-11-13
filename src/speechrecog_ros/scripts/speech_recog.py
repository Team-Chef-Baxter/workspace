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
    if not OPENAI_API_KEY:
        rospy.logerr("OpenAI API key not found.")
        sys.exit(1)
    openai.api_key = OPENAI_API_KEY

    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
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

def main():
    rospy.init_node('voice_assistant_node')
    speech_pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    response_pub = rospy.Publisher('assistant_response', String, queue_size=10)
    recipe_pub = rospy.Publisher('salad_recipe', SaladRecipe, queue_size=10)

    recognizer = sr.Recognizer()
    DEVICE_INDEX = 9  
    microphone = sr.Microphone(device_index=DEVICE_INDEX)

    #engine = pyttsx3.init()
    #engine.setProperty('rate', 150)
    #engine.setProperty('volume', 1.0)
    use_hardcoded_ingredients = False #change to true if u wanna use

    rospy.loginfo("Welcome to Baxter the Salad Assistant!")
    rospy.loginfo("You can ask me anything about making salads.")
    rospy.loginfo("Say 'exit', 'quit', or 'stop' to end the conversation.")

    messages=[
                {"role": "system", "content": (
                    "You are a helpful assistant who can engage in casual conversation and provides salad recipes and instructions."
                    "Only when the user explicitly asks for your help to make a salad(e.g., 'Help me make a Greek salad', 'Can you make a salad for me') provide a step by step recipe and instruction include the recipe in the following JSON format below, enclosed between '<BEGIN_JSON>' and '<END_JSON>' tags. If the user just asks how to make a certain salad, give them a natural language recipe."
                    "When the user wants to make a salad but hasn't specified ingredients, ask them if they have the necessary ingredients. If they say anything along the lines of 'yes', provide the recipe and instructions in natural language."
                    "If the user says they have certain ingredients (e.g., 'I have lettuce, tomatoes and cucumbers'), suggest a salad that can be made with those ingredients and ask 'You could make a [] salad with those ingredients. Do you want a recipe and instructions? If the user says anything along the lines of 'Yes', provide the recipe and instructions in natural language."
                    "All recipes and instructions should be in the JSON format:\n\n"
                    "<BEGIN_JSON>\n"
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
                    "<END_JSON>\n\n"
                    "Remember that users may ask you to make the salad for them within a conversation, and if they do so, provide a step by step recipe and instruction include the recipe in the JSON format, enclosed between '<BEGIN_JSON>' and '<END_JSON>' tags."
                    "Do not include 'gather ingredients' as a command in the JSON. Also, all motions should only be in the form of 'pick up', 'place into', 'stir', 'mix'. Exclude all other motions that involve a knife or water(e.g. 'dice', 'chop', 'wash'), and assume ingredients needing chopping or slicing are already prepared as so."
                    "Only use the JSON format when providing recipes or instructions. For all other queries or casual conversation, respond in natural language. And, after providing the JSON format, the next message on your end should be 'Do you need any other help' and if the user says 'no', you can terminate the conversation."
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

        speak_text(assistant_response)

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
