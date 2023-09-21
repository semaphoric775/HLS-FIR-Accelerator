import re, os, sys

module=sys.argv[1]
clk_per=sys.argv[2]

results=open('results.csv','a')

areaScores=['MUX', 'FUNC', 'LOGIC', 'BUFFER', 'MEM', 'ROM', 'REG', 'FSM-REG', 'FSM-COMB']
areaScoreDict = {}
for cat in areaScores:
  areaScoreDict[cat]='0.0'

# date__begin
results.write(open('hls.begin').readlines()[0].strip()+',')
# date__end
results.write(open('hls').readlines()[0].strip()+',')
# module_name
results.write(module+',')
# clk_per
results.write(clk_per+',')

counting_components=False
f=open(f"Catapult/{module}.v1/rtl.rpt")
for line in f:
  m=re.search(r"^\s*Design Total:\s+(\S+)\s+(\S+)\s+(\S+)",line)
  if m:
    # realops
    results.write(m.group(1)+',')
    # latency
    results.write(m.group(2)+',')
    # throughput
    results.write(m.group(3)+',')
    continue
  m=re.search(r"^\s*([A-Z\-]+):.*\s([0-9\.]+)\s+\([0-9\.]+%\)\s*$",line)
  if m:
    cat=m.group(1)
    asc=m.group(2)
    # print(cat,asc)
    if cat in areaScores:
      areaScoreDict[cat]=asc
  m=re.search(r"^\s*Max Delay:\s+(\S+)",line)
  if m:
    # critpath
    results.write(m.group(1)+',')
    continue
f.close()

for cat in areaScores[:-1]:
  results.write(areaScoreDict[cat]+',')
results.write(areaScoreDict[cat]+'\n')

results.close()

