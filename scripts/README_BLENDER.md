# Blender에서 AUV 메쉬 생성하기

## 1. 윈도우에서 Blender 실행

### 준비물:
- Blender 3.0 이상 (https://www.blender.org/download/)
- `generate_auv_meshes.py` 파일

## 2. 사용 방법

### Step 1: Blender 열기
1. Blender 실행
2. 기본 씬이 열림 (Cube, Camera, Light)

### Step 2: Scripting 모드로 전환
1. 상단 메뉴에서 **"Scripting"** 탭 클릭
2. 화면이 스크립트 편집기로 전환됨

### Step 3: 스크립트 불러오기
1. 텍스트 편집기 영역에서 **"Open"** 버튼 클릭
2. `generate_auv_meshes.py` 파일 선택
3. 스크립트가 편집기에 표시됨

### Step 4: 출력 경로 설정 (선택사항)
스크립트 상단 부근에서 출력 경로를 원하는 위치로 변경:
```python
# 기본값: 바탕화면에 auv_meshes 폴더 생성
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop", "auv_meshes")

# 원하는 경로로 변경 가능 (예시)
OUTPUT_DIR = "C:/Users/YourName/Documents/auv_meshes"
```

### Step 5: 스크립트 실행
1. 텍스트 편집기에서 **▶ "Run Script"** 버튼 클릭
   - 또는 단축키: **Alt + P**
2. 하단 시스템 콘솔에서 진행 상황 확인
3. 완료되면 "All mesh files exported" 메시지 표시

### Step 6: 생성된 파일 확인
설정한 출력 경로에서 다음 파일들 확인:
- `custom_auv_hull.obj` - 메인 본체
- `custom_auv_nose.obj` - 노즈 콘
- `custom_auv_tail.obj` - 테일 콘
- `custom_auv_fin.obj` - 제어 날개
- `custom_auv_duct.obj` - 프로펠러 덕트

## 3. WSL로 파일 전송

### Windows → WSL 경로 변환
윈도우 경로: `C:\Users\YourName\Desktop\auv_meshes`
WSL 경로: `/mnt/c/Users/YourName/Desktop/auv_meshes`

### 파일 복사
WSL 터미널에서:
```bash
# 윈도우에서 생성한 파일 복사
cp /mnt/c/Users/YourName/Desktop/auv_meshes/*.obj ~/projectAlpha/src/stonefish_ros2/data/

# 또는 심볼릭 링크가 있는 경우
cp /mnt/c/Users/YourName/Desktop/auv_meshes/*.obj ~/projectAlpha/stonefish/Library/data/
```

## 4. 메쉬 확인 (Blender에서)

생성된 메쉬를 확인하려면:
1. **File → Import → Wavefront (.obj)**
2. 생성된 OBJ 파일 선택
3. 3D Viewport에서 모델 확인
4. 필요시 수정 후 다시 Export

## 5. 메쉬 수정 팁

### 크기 조정:
```python
# 스크립트에서 scale 파라미터 변경
radius=0.095  # 반지름 조정
depth=1.3     # 길이 조정
```

### 세밀도 조정:
```python
vertices=32   # 원통의 면 개수 (높을수록 부드러움)
levels=2      # Subdivision 레벨 (높을수록 부드러움)
```

### 날개 모양 변경:
```python
# create_control_fin() 함수에서 verts 좌표 수정
verts = [
    (-0.075, 0, 0.005),  # 앞쪽 루트
    (0.075, 0, 0.005),   # 뒤쪽 루트
    # ... 좌표 조정
]
```

## 6. 문제 해결

### 스크립트 실행 오류:
- **"bpy module not found"**: Blender 외부에서 실행하려 함 → Blender 내부에서 실행
- **Export 함수 오류**: Blender 버전 확인 (3.0 이상 권장)

### 파일이 생성되지 않음:
- 출력 경로 권한 확인
- 경로에 한글이 없는지 확인
- 시스템 콘솔(Window → Toggle System Console)에서 에러 메시지 확인

### 메쉬가 이상하게 보임:
- Blender에서 Import해서 확인
- `bpy.ops.object.shade_smooth()` 적용 여부 확인
- Subdivision 레벨 조정

## 7. 다음 단계

메쉬 파일 생성 후:
1. WSL로 파일 복사
2. ROS2 패키지 빌드: `colcon build --packages-select stonefish_ros2`
3. 시뮬레이션 실행: `ros2 launch stonefish_ros2 custom_auv_test.launch.py`
4. 부력/무게 조정 (시나리오 파일에서)
5. 제어 테스트

## 참고 사항

- **좌표계**: Stonefish는 X-forward, Z-up 사용
- **단위**: 모든 치수는 미터(m) 단위
- **파일 크기**: 메쉬가 너무 복잡하면 시뮬레이션 성능 저하
  - Vertices: 1000~5000개 권장
  - Faces: 2000~10000개 권장