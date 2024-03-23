class ReportsHash:
    def __init__(self, value, name):
        self._value = value
        self._name = name

    def __hash__(self):
        hash_value = hash(self._value)
        print(f'Calling hash for {self._name}: {hash_value}')
        return hash_value

    def __eq__(self, other):
        return isinstance(other, ReportsHash) and self._value == other._value
    
    def __repr__(self):
        return f"{self._value, self._name}"
    
a = ReportsHash(42, 'a')
b = ReportsHash(37, 'b')
c = ReportsHash(42, 'c')
d = ReportsHash(1, 'c')

s = {a, b}

print(c in s)
s.add(c)

if c in s:
    s.remove(c)
    s.add(c)
    
for k in s:
    print(k)

s.remove(d)