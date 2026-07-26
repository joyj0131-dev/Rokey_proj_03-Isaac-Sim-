"""matplotlib 3D projection이 깨지는 환경 문제를 우회한다.

이 컴퓨터에는 옛 apt matplotlib(3.5.1)이 설치한 mpl_toolkits-nspkg.pth가
남아 있어서, 파이썬 시작 시점에 mpl_toolkits를 잘못된(구버전) 경로로 미리
캐시해버린다. sys.modules에 캐시되면 이후 import는 sys.path를 다시 보지
않으므로, matplotlib을 import하기 *전에* 이 캐시를 지워야 한다.

사용법: 다른 어떤 것보다도 먼저(특히 matplotlib import 전에) 이 모듈을
import할 것: `import _fix_mpl3d` (부작용만 있고 이름은 안 씀).
"""
import sys

sys.modules.pop("mpl_toolkits", None)
sys.path = [p for p in sys.path if p != "/usr/lib/python3/dist-packages"]
