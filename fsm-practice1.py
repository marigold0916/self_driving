class InitializationError(Exception):
    pass

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except KeyError:
            raise InitializationError("must call .set_start() before run()")
        if not self.endStates:
            raise InitializationError("at least one state must be an end_state")
        while True:
            (newState, cargo) = handler(cargo)
            newState = newState.upper()
            if newState in self.endStates:
                print("Reached", newState)
                break
            else:
                handler = self.handlers[newState]

# 긍정/부정 형용사 리스트
pos_adj = ["great", "super", "fun", "entertaining", "easy"]
neg_adj = ["boring", "difficult", "ugly", "bad"]

# 상태 처리 함수들
def start_transitions(txt):
    words = txt.split(None, 1)
    word, txt = words if len(words) > 1 else (txt, "")
    if word == "Python":
        return "python_state", txt
    return "error_state", txt

def python_state_transitions(txt):
    words = txt.split(None, 1)
    word, txt = words if len(words) > 1 else (txt, "")
    if word == "is":
        return "is_state", txt
    return "error_state", txt

def is_state_transitions(txt):
    words = txt.split(None, 1)
    word, txt = words if len(words) > 1 else (txt, "")
    if word == "not":
        return "not_state", txt
    elif word in pos_adj:
        return "neg_state", txt  # 긍정 단어: 부정 상태로 전이
    elif word in neg_adj:
        return "pos_state", txt  # 부정 단어: 긍정 상태로 전이
    return "error_state", txt

def not_state_transitions(txt):
    words = txt.split(None, 1)
    word, txt = words if len(words) > 1 else (txt, "")
    if word in pos_adj:
        return "pos_state", txt  # not + 긍정 단어 → 부정
    elif word in neg_adj:
        return "neg_state", txt  # not + 부정 단어 → 긍정
    return "error_state", txt

# 상태 기계 인스턴스 구성
m = StateMachine()
m.add_state("Start", start_transitions)
m.add_state("Python_state", python_state_transitions)
m.add_state("is_state", is_state_transitions)
m.add_state("not_state", not_state_transitions)
m.add_state("neg_state", lambda txt: ("neg_state", ""), end_state=1)
m.add_state("pos_state", lambda txt: ("pos_state", ""), end_state=1)
m.add_state("error_state", lambda txt: ("error_state", ""), end_state=1)
m.set_start("Start")

# 테스트 실행
test_sentences = [
    "Python is great",       # 긍정
    "Python is not bad",     # 긍정 (이중 부정)
    "Python is difficult",   # 부정
    "Perl is ugly",          # 오류
]

for sentence in test_sentences:
    print(f"Input: {sentence}")
    m.run(sentence)
    print("-" * 30)
