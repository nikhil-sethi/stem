import torch
from sklearn.metrics.pairwise import cosine_similarity
import time
import numpy as np

# Load CLIP model
device = "cuda" if torch.cuda.is_available() else "cpu"

model_name = "bert" # bert, clip
if model_name=="clip":
    import clip
    model, preprocess = clip.load("ViT-B/32", device=device)

    def get_embedding(text):
        text_inputs = clip.tokenize([text]).to(device)
        with torch.no_grad():
            text_features = model.encode_text(text_inputs)
        return text_features.cpu().numpy()

elif model_name =="bert":
    from transformers import BertTokenizer, BertModel
    tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
    model = BertModel.from_pretrained('bert-base-uncased')

    def get_embedding(text):
        inputs = tokenizer(text, return_tensors='pt')
        with torch.no_grad():
            outputs = model(**inputs)
        # Get the mean of the last hidden state
        embeddings = torch.mean(outputs.last_hidden_state, dim=1)
        return embeddings.detach().numpy()

def add_context(label, context):
    return label + context

# Example contexts and items
target = "human after earthquake"

clues = [
    target,
    "helmet",
    "rope",
    "blood",
    "backpack",
    "drill",
    "phone",
    "fire extinguisher"
]

clues = [
    target,
    "helmet",
    "rope",
    "blood",
    "bag",
    "chair",
    "table",
    "fire extinguisher"
]


context = ""

contextual_clues = [add_context(clue, context) for clue in clues]

start = time.perf_counter()


# Get embeddings
embeddings = [get_embedding(clue) for clue in contextual_clues]

# Calculate cosine similarity
scores = {label: cosine_similarity(embeddings[0], emb) for label, emb in zip(clues, embeddings)}
print("Time: ", time.perf_counter()-start)

print(f"Cosine Similarity to target: {target} in context: {context} ")
for key, val in scores.items():
    print(f"{key}: {val[0][0]}")
