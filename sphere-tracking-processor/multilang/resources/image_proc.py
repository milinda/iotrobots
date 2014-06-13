import storm

class SplitSentenceBolt(storm.BasicBolt):
    def process(self, tup):
        storm.emit(tup.values[0])

SplitSentenceBolt().run()