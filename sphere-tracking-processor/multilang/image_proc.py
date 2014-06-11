import storm

class SplitSentenceBolt(storm.BasicBolt):
    def process(self, tup):
        print tup.values[0]

SplitSentenceBolt().run()