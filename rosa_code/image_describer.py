import base64
from openai import OpenAI

# Initialize OpenAI client
client = OpenAI()

def analyze_image(image_path, model="gpt-4o-mini"):
    """
    Analyze an image using the OpenAI API and return the content of the response.

    Args:
        image_path (str): Path to the image file (.png or .jpg).
        model (str): The model to use for the analysis.

    Returns:
        str: The content of the assistant's response.
    """
    # Encode the image as base64
    def encode_image(path):
        with open(path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    # Determine MIME type based on file extension
    if image_path.lower().endswith(".png"):
        mime_type = "image/png"
    elif image_path.lower().endswith(".jpg") or image_path.lower().endswith(".jpeg"):
        mime_type = "image/jpeg"
    else:
        raise ValueError("Unsupported file format. Please use .png, .jpg, or .jpeg.")

    # Encode the image
    base64_image = encode_image(image_path)

    # Send the request to the OpenAI API
    response = client.chat.completions.create(
        model=model,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "What is in this image?",
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{mime_type};base64,{base64_image}"
                        },
                    },
                ],
            }
        ],
    )

    # Return just the content
    return response.choices[0].message.content
