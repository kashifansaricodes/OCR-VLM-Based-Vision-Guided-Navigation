from scripts.inference import inference_once
import matplotlib.pyplot as plt
from PIL import Image
import os
import sys
from io import StringIO

# model_path = "mtgv/MobileVLM-1.7B" # MobileVLM
model_path = "mtgv/MobileVLM_V2-1.7B" # MobileVLM V2
image_file = "assets/samples/isaacsim_test_image6.png"
prompt_str = "what sign do you see?"

# Display the image, prompt and answer
def display_image_prompt_and_answer(image_path, prompt, model_name, answer):
    # Load and display the image
    img = Image.open(image_path)
    plt.figure(figsize=(10, 8))
    plt.imshow(img)
    
    # Add prompt as title and model info as subtitle
    plt.title(f"Prompt: {prompt}", fontsize=20)
    
    # Add answer below the image
    plt.figtext(0.5, 0.08, f"Answer: {answer}", 
                ha="center", fontsize=17, bbox={"facecolor":"lightyellow", "alpha":0.8, "pad":2})
    
    # Add model info at the bottom
    plt.figtext(0.5, 0.01, f"Using model: {model_name}", 
                ha="center", fontsize=10, bbox={"facecolor":"lightgrey", "alpha":0.5, "pad":5})
    
    plt.axis('on')  
    plt.tight_layout()
    plt.show()

def capture_model_output(args):
    """Capture the output from inference_once which might be printing to stdout"""
    # Store the original stdout
    old_stdout = sys.stdout
    # Create a string buffer to capture output
    captured_output = StringIO()
    # Redirect stdout to our capture buffer
    sys.stdout = captured_output
    
    try:
        # Run the inference function
        inference_once(args)
        # Get the captured output
        output = captured_output.getvalue().strip()
        
        # Extract the model's answer (assuming it's after "🚀 MobileVLM" in the output)
        if "🚀" in output:
            answer_text = output.split("🚀")[1].split(":")[1].strip()
        else:
            answer_text = output
            
        return answer_text
    finally:
        # Restore the original stdout
        sys.stdout = old_stdout

args = type('Args', (), {
    "model_path": model_path,
    "image_file": image_file,
    "prompt": prompt_str,
    "conv_mode": "v1",
    "temperature": 0, 
    "top_p": None,
    "num_beams": 1,
    "max_new_tokens": 15,
    "load_8bit": False,
    "load_4bit": False,
})()

# Run inference and capture the answer
answer = capture_model_output(args)
print(f"Answer: {answer}")

# Display the image, prompt, and answer
display_image_prompt_and_answer(image_file, prompt_str, os.path.basename(model_path), answer)