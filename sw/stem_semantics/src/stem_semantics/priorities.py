
from sklearn.metrics.pairwise import cosine_similarity
import time
import numpy as np

def create_label_map(file):
    with open(file, 'r') as f:
        labels = f.readlines()
        return {labels[i][:-1]:i for i in range(len(labels))}

class BasePriorityMap():
    def __init__(self, target, context, p_max) -> None:
        self.target = target
        self.situation = context
        self.label_map = create_label_map("/workspaces/stem_ws/src/thesis/sw/stem_semantics/config/labels.txt") #TODO make this relative later on
        self.objects = list(self.label_map.keys())
        self.p_max = p_max
        self.p_min = 1 # exploration baseline

    def __call__(self, label):
        return self.priorities[self.label_map[label]]

class SemanticPriorityMap(BasePriorityMap):
    
    def __init__(self, target = "human", situation = "mine", p_max = 10, model_name = "bert") -> None:

        super().__init__(target, situation, p_max)

        import torch
        self.model_name = model_name

        device = "cuda" if torch.cuda.is_available() else "cpu"
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

            tokenizer = BertTokenizer.from_pretrained('bert-large-uncased')
            model = BertModel.from_pretrained('bert-large-uncased')

            def get_embedding(text):
                inputs = tokenizer(text, return_tensors='pt')
                with torch.no_grad():
                    outputs = model(**inputs)
                # Get the mean of the last hidden state
                embeddings = torch.mean(outputs.last_hidden_state, dim=1)
                return embeddings.detach().numpy()

        self.embedding_func = get_embedding

        self.objects = self.prepare_sequence()
        self.embeddings = self.create_embeddings(self.objects)
        self.cosim_list = self.create_cosim_list(self.embeddings)
        # print(self.cosim_list)
        self.priorities = self.scale_scores(self.cosim_list)

    def prepare_sequence(self):
        target_in_context = self.target + " in " + self.situation
        self.objects.insert(0, target_in_context)
        return self.objects
    
    def create_embeddings(self, objects):
        embeddings = [self.embedding_func(object) for object in objects]
        return embeddings
    
    def create_cosim_list(self, embeddings):
        # only objects are taken for embeddings because the target skews up the cosine scale
        scores = np.array([cosine_similarity(embeddings[0], emb)[0][0] for emb in embeddings[2:]])
        return scores
    
    def scale_scores(self, scores):
        priorities = self.p_min +  (self.p_max - self.p_min)*( (scores-min(scores))/(max(scores)-min(scores)) ) 
        return np.round(priorities).astype(int)

class GTPriorityMap(BasePriorityMap):
    """Convenience class for offline inferred values. Generated using SemanticPriorityMap """
    def __init__(self, target, context, p_max) -> None:
        super().__init__(target, context, p_max)
        # inferred from the semantic priority map for the human target, and stored offline for convenience
        if "earthquake" in context :
            self.priorities = [1, 8, 5, 5, 6, 5, 6, 5, 5, 7, 8, 6, 4, 4, 7, 4, 1, 4, 2, 4, 5, 4]
            # self.priority_map = {                     
            #     "human": 8,
            #     "helmet":1,
            #     "blood":8,
            #     "chair":5,
            #     "table":5,
            #     "gun":6,
            #     "drill":5,
            #     "phone":6,
            #     "rubble":5,
            #     "flashlight":5,
            #     "radio":7,
            #     "dog":8,
            #     "worker":6,
            #     "knife":4,
            #     "wall":4,
            #     "ground":7,
            #     "toy":4,
            #     "ball":1,
            #     "rope":4,
            #     "headphones":2,
            #     "sofa":4,
            #     "carpet":5,
            #     "plant" :4
            # }

        elif ("mine" in context) or ("cave" in context):
            self.priorities = [1, 8, 4, 4, 4, 4, 6, 4, 4, 6, 7, 6, 3, 3, 6, 3, 1, 3, 3, 3, 4, 4]                     
            # self.priority_map = {                     
                #     "human": 8,
                #     "helmet":1,
                #     "blood":8,
                #     "chair":4,
                #     "table":4,
                #     "gun":4,
                #     "drill":4,
                #     "phone":6,
                #     "rubble":4,
                #     "flashlight":4,
                #     "radio":6,
                #     "dog":7,
                #     "worker":6,
                #     "knife":3,
                #     "wall":3,
                #     "ground":6,
                #     "toy":3,
                #     "ball":1,
                #     "rope":3,
                #     "headphones":3,
                #     "sofa":3,
                #     "carpet":4,
                #     "plant" :4,
                # }
        elif "amrlab" in context:
            self.priorities = [1, 8, 5, 8, 4, 4, 6, 4, 4, 6, 7, 6, 3, 3, 6, 3, 1, 3, 3, 3, 4, 4]                     
            # self.priority_map = {                     
                #     "human": 8,
                #     "helmet":1,
                #     "blood":8,
                #     "chair":4,
                #     "table":7,
                #     "gun":4,
                #     "drill":4,
                #     "phone":6,
                #     "rubble":4,
                #     "flashlight":4,
                #     "radio":6,
                #     "dog":7,
                #     "worker":6,
                #     "knife":3,
                #     "wall":3,
                #     "ground":6,
                #     "toy":3,
                #     "ball":1,
                #     "rope":3,
                #     "headphones":3,
                #     "sofa":3,
                #     "carpet":4,
                #     "plant" :4,
                # }

        self.priorities.insert(0, 0) #  0 means empty space. just to sit right with label numbering.
        self.priorities.insert(1, p_max) # manually add max priority for the target. human in this hardcode. not included in inference because it skews distribution. Does not affect anything, just convenience to get a better spread.

        self.priorities = np.array(self.priorities)

if __name__=="__main__":
    map = SemanticPriorityMap(p_max=8, situation="an underground mine", target="human")
    print(map.priorities)
    sorted_priorities = np.sort(map.priorities)[::-1]
    objects = np.array(map.objects[2:])[np.argsort(map.priorities)[::-1]]
    print(f"Cosine Similarity to target: {map.target} in context: {map.situation} ")
    for key, val in zip(objects, sorted_priorities):
        print(f"{key}: {val}")
