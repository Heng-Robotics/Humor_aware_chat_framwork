#-*-coding:utf-8-*-

import speech_recognition as sr
# from spellchecker import SpellChecker

class SpeechRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.language = "en-US"
        self.retries = 1
        # self.spell = SpellChecker()

    def listen_and_recognize(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            print("Please say something:")
            audio = self.recognizer.listen(source)
            print("Recognizing Now .... ")

            for attempt in range(self.retries):
                try:
                    recognized_text = self.recognizer.recognize_google(audio, language=self.language)
                    corrected_text = recognized_text
                    # corrected_text = self.correct_spelling(recognized_text)########
                    return corrected_text
                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand the audio.")
                except sr.RequestError as e:
                    print("Could not request results from Google Speech Recognition service; {0}".format(e))
                except Exception as e:
                    print("There is an error in robot_speech_recognition/listen_and_recognize: " + str(e))
            return None

    # def correct_spelling(self, text):
    #     words = text.split()
    #     corrected_words = [self.spell.correction(word) for word in words]
    #     return ' '.join(corrected_words)


if __name__ == "__main__":
    recognizer = SpeechRecognizer()
    response = recognizer.listen_and_recognize()
