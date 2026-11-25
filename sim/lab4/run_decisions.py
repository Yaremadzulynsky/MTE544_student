"""Run the unchanged LabFour decision node with an explicit A* heuristic."""
import argparse,functools,runpy,sys
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--heuristic',choices=['euclidean','manhattan'],default='euclidean');args=p.parse_args()
sys.path.insert(0,str(REPO))
import planner
planner.search=functools.partial(planner.search,heuristic=args.heuristic)
sys.argv=['decisions.py']
runpy.run_path(str(REPO/'decisions.py'),run_name='__main__')
