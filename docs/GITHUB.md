# Публикация проекта на GitHub

Пароль GitHub и токены не нужно вставлять в исходники, README или переписку.
При первой отправке Git Credential Manager предлагает войти в GitHub через браузер.
Обычный пароль учётной записи не используется как пароль HTTPS Git.

## Через установленный Git

1. Войдите в GitHub и откройте https://github.com/new.
2. Укажите имя, например `metro-lidar-obstacle-detector`.
3. Выберите Private или Public. Для приватного проекта дайте проверяющим доступ через Settings → Collaborators.
4. Создайте пустой репозиторий: не включайте генерацию README, .gitignore или лицензии, поскольку проект уже содержит свои файлы.
5. Откройте PowerShell в папке проекта и выполните:

```powershell
Set-Location D:\lidar\metro-obstacle-detector
git status
git remote add origin https://github.com/YOUR_LOGIN/YOUR_REPOSITORY.git
git push -u origin main
```

Замените `YOUR_LOGIN` и `YOUR_REPOSITORY`. Вход выполните в открывшемся браузере.
Если `origin` уже существует, сначала посмотрите `git remote -v`. Меняйте его через
`git remote set-url origin URL` только если это нужный репозиторий.

Если запускаете инструкцию из скачанных исходников без `.git`, сначала:

```powershell
git init -b main
git add .
git status
git commit -m "Release METRO LiDAR detector 0.4.2"
```

Для коммита нужны `git config user.name "Your name"` и `git config user.email "Your email"`.
При желании используйте свой подтверждённый GitHub noreply email из Settings → Emails.
Не выполняйте `push --force`, если в удалённом репозитории уже есть чужие изменения.

## Через GitHub CLI

Установите CLI с https://cli.github.com и выполните:

```powershell
gh auth login --hostname github.com --git-protocol https --web
gh repo create YOUR_LOGIN/YOUR_REPOSITORY --private --source . --remote origin --push
```

Для открытой публикации замените `--private` на `--public`. Этот вариант создаёт
новый репозиторий. Для существующего используйте `git remote add` и `git push` из первого варианта.

## Состав публикации

Исходный код, Dockerfile, настройки, локальные web-зависимости с лицензией,
тесты, документация и сводные результаты проверок. `.gitignore` исключает
датасеты, state, секреты окружения, временные эксперименты и тяжёлые архивы.
Полные покадровые JSONL входят в архив поставки, а не в исходный Git-репозиторий.

Готовый Docker-образ `metro-detector-image.tar.gz` можно передать отдельно либо
приложить к GitHub Release. Не добавляйте его обычным `git add`.
Проверяющий также может собрать образ по Dockerfile.

После отправки убедитесь, что GitHub показывает README, папки `metro_detector`,
`web`, `tests`, `docs`, файл `Dockerfile`, а папок `state` и `work` нет.

Источники: [GitHub — публикация локального проекта](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github),
[GitHub CLI — создание репозитория](https://cli.github.com/manual/gh_repo_create).
