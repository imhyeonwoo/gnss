# 🚀 Jeju Autonomous Driving Project - Git 사용 가이드

이 문서는 **Jeju Autonomous Driving Project**에서 **GitHub에 콘텐츠를 올리고 관리하는 방법**을 정리한 가이드입니다.  
이제 VSCode 터미널을 사용하여 GitHub에 원화할 수 있습니다. ✅

---

## 📌 1. 변경 상황을 GitHub에 업로드하는 방법

### 🔹 1️⃣ VSCode에서 프로젝트 열기
1. **VSCode 실행 후** `Ctrl + ~` (백트흽) 키를 누르여 **터미널 창**을 여는다.
2. 터미널에서 프로젝트 디렉토리로 이동한다.
   ```bash
   cd ~/git/gnss
   ```

### 🔹 2️⃣ 현재 상황 확인 (`git status`)
```bash
git status
```
> 현재 변경된 파일이 있는지 확인할 수 있습니다.

### 🔹 3️⃣ 변경된 파일 추가 (`git add`)
모든 변경 상황을 Git에 추가한다.
```bash
git add .
```
> **📌 TIP:** 특정 파일만 추가하려면 `git add 파일명` 을 사용하세요.  
> 예: `git add src/control/controller.cpp`

### 🔹 4️⃣ 컬리트 작성 (`git commit`)
변경 상황을 설명하는 메시지를 작성하여 컬리트한다.
```bash
git commit -m "설명: 수정한 내용 요약"
```
✅ **예제:**
```bash
git commit -m "Fix: GPS 좌표 변환 오류 수정"
git commit -m "Add: GlobalYawEstimator 노드에 EMA 필터 적용"
```

### 🔹 5️⃣ GitHub에 업로드 (`git push`)
```bash
git push origin main
```
이제 변경 상황이 GitHub에 반영됩니다! 🚀

---

## 📌 2. 최신 콘텐츠 가져오는 방법 (`git pull`)
다른 사람이 작성한 최신 콘텐츠를 가져오려면:
```bash
git pull origin main
```
⚠️ **주의:**  
마릇 수정한 내용이 있는데 `git pull`을 하면 **충돌(conflict)** 이 발생할 수 있습니다.  
이 경우 직접 해결 후 `git add . && git commit -m "Conflict 해결"` 을 실행하세요.

---

## 📌 3. 새로운 기능 개발을 위한 브랜치 관리
새로운 기능을 추가할 때는 **브랜치를 생성하여 작업**하는 것이 좋습니다.

### 🔹 1️⃣ 새로운 브랜치 생성 및 이동
```bash
git checkout -b feature/new-feature-name
```
> 예: `git checkout -b feature/path-planning`

### 🔹 2️⃣ 브랜치에서 작업 후 컬리트 & 퓨시
```bash
git add .
git commit -m "설명: 새로운 기능 추가"
git push origin feature/new-feature-name
```

### 🔹 3️⃣ PR(Pull Request) 생성 후 별표하기
GitHub에서 PR(Pull Request)을 생성하여 `main` 브랜치로 별표하세요.
