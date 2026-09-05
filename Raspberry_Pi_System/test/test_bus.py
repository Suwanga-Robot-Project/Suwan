import sys

print("1. Loading PIL.Image...")
import PIL.Image

print("2. Loading google.genai...")
from google import genai

print("3. Initializing Gemini Client...")
try:
    client = genai.Client(api_key="TEST_KEY")
    print(" -> Gemini Client initialized.")
except Exception as e:
    print(f" -> GenAI Init Error: {e}")

print("4. Initializing I2C Bus (Hardware)...")
import board
try:
    i2c = board.I2C()
    print(" -> I2C Bus initialized.")
except Exception as e:
    print(f" -> I2C Error: {e}")

print("? test_bus2 completed successfully!")