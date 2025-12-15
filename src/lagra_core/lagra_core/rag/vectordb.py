import chromadb
from chromadb.utils import embedding_functions

class ChromaStore:
    def __init__(self, persist_dir="/tmp/lagra_chroma"):
        self.client = chromadb.PersistentClient(path=persist_dir)
        self.col = self.client.get_or_create_collection("lagra_memory")

    def add(self, doc_id: str, text: str, embedding):
        self.col.add(ids=[doc_id], documents=[text], embeddings=[embedding.tolist()])

    def query(self, embedding, k=5):
        res = self.col.query(query_embeddings=[embedding.tolist()], n_results=k)
        return res.get("documents", [[]])[0]
