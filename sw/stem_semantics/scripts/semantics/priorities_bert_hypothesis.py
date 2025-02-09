from transformers import pipeline

# Define the context, target, and objects
context = "Search and rescue after an earthquake"
target = "Human"
objects = [
    "human", "helmet", "gas mask", "blood", "chair", "table", "gun", "drill", 
    "phone", "rubble", "medical kit", "water bottle", "survival blanket", 
    "flashlight", "robot", "radio", "dog", "worker", "knife", "wall", "ground"
]

# Load a pre-trained model and tokenizer from Hugging Face
model_name = "distilbert-base-uncased"
nlp = pipeline("zero-shot-classification", model=model_name)

# Construct the hypothesis templates
hypothesis_template = "This object is related to a {} in the context of {}."

# Generate the hypotheses
hypotheses = [hypothesis_template.format(obj, context) for obj in objects]

# Get predictions for each hypothesis
predictions = nlp(
    sequences=hypotheses,
    candidate_labels=[target],
    multi_label=True
)

# Rank the objects based on the model's scores
scores = {obj: pred['scores'][0] for obj, pred in zip(objects, predictions)}
ranked_objects = sorted(scores, key=scores.get, reverse=True)

# Print the ranked objects
print("Ranked Objects:\n", ranked_objects)
