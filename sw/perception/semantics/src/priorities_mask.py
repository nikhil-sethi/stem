from transformers import pipeline, AutoTokenizer, AutoModelForMaskedLM
from pprint import pprint
import torch

# Load the pretrained model and tokenizer
model_name = "bert-large-uncased"  # or any other MLM model
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForMaskedLM.from_pretrained(model_name)

def get_embedding(text, model, tokenizer):
    inputs = tokenizer(text, return_tensors='pt')
    with torch.no_grad():
        outputs = model(**inputs)
    # Get the mean of the last hidden state
    embeddings = torch.mean(outputs.last_hidden_state, dim=1)
    return embeddings.detach().numpy()


# Define your custom list of tokens
filter_tokens = ["object", "human"]

# Define your input sentence with a [MASK] token
input_sentence = "I want to [MASK] a pizza."

target = "person"
context = "earthquake"

# input_sentence = f"To find a {target} in a {context}, I should first try to find a [MASK]."
input_sentence = f"In a {context} situation, there is likely to be an object that looks like a [MASK] near the {target}."
# Tokenize the input sentence
tokenized_input = tokenizer(input_sentence, return_tensors="pt")

# Create a pipeline for masked language modeling
mlm_pipeline = pipeline("fill-mask", model=model, tokenizer=tokenizer)

predictions = mlm_pipeline(input_sentence, top_k=20)
pprint(predictions)
# Iterate over custom tokens, replacing [MASK] with each token and generating predictions
for pred in predictions:
    # filter based on similarity
    token_embed = 
    # Get the score for the predicted token
    score = 0
    for pred in predictions:
        if pred['token_str'] == token:
            score = pred['score']
            break
    print(f"Token: {token}, Score: {score}")
