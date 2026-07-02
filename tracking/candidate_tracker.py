import math

class CandidateManager:
    def __init__(self):
        self.reset()

    def reset(self):
        self.candidates = {}
        self.next_candidate_id = 0

    def process_detection(self, cx, cy, box, score_modifier):
        """Updates existing candidates or registers a new one based on distance."""
        matched = False
        for cid, cdata in self.candidates.items():
            if math.hypot(cx - cdata['center'][0], cy - cdata['center'][1]) < 75: 
                self.candidates[cid].update({'center': (cx, cy), 'box': box})
                self.candidates[cid]['score'] += score_modifier
                matched = True
                break
                
        if not matched:
            self.candidates[self.next_candidate_id] = {'box': box, 'center': (cx, cy), 'score': 0}
            self.next_candidate_id += 1

    def get_locked_target(self, threshold=10):
        """Returns the bounding box of the winning candidate, or None."""
        if not self.candidates: 
            return None, 0
            
        best_id = max(self.candidates, key=lambda k: self.candidates[k]['score'])
        best_score = self.candidates[best_id]['score']
        
        if best_score > threshold:
            return self.candidates[best_id]['box'], best_score
        return None, best_score