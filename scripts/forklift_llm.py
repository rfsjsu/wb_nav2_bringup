import os

import dotenv
from langchain_openai import ChatOpenAI
from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama

llm_vendor = "Anthropic"

def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance."""
    dotenv.load_dotenv(dotenv.find_dotenv())


   # Get the current username
    user_name = os.getenv("USER")

    try:
        #if user_name == "ros":
        if llm_vendor == "Ollama":
            print("Using local Ollama")
            llm = ChatOllama(
                model="qwen3.5:4b",
                temperature=0.0,
                base_url="http://localhost:11434",  # Can be changed for remote Ollama instances
            )
        elif(llm_vendor == "Anthropic"):
            print("Using Anthropic")
            llm = ChatAnthropic(
                model=get_env_variable("ANTHROPIC_MODEL"),
                temperature=0,
                anthropic_api_key=get_env_variable("ANTHROPIC_API_KEY"),
                max_tokens=4096,
            )
        elif (llm_vendor == "OpenAI"):
            print("Using OpenAI")
            llm = ChatOpenAI(
                api_key=get_env_variable("OPENAI_API_KEY"),
                model="gpt-4o",
                streaming=streaming,
            )
    except Exception as e:
        print(f"Error initializing LLM: {e}")
        return

    return llm


def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name)
    if value is None:
        msg = f"Environment variable {var_name} is not set."
        raise ValueError(msg)
    return value
