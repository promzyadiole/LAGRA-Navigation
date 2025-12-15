from openai import AzureOpenAI

import os

AZURE_API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
if AZURE_API_KEY is None:
    raise RuntimeError("AZURE_OPENAI_API_KEY not set")


response = client.chat.completions.create(
    messages=[
        {
            "role": "system",
            "content": "You are a helpful assistant.",
        },
        {
            "role": "user",
            "content": "Please give me an introduction onto Large Language Models",
        }
    ],
    max_tokens=16384,
    temperature=1.0,
    top_p=1.0,
    model="gpt-5-chat"
)

print(response.choices[0].message.content)
