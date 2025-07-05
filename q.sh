# если пайп ещё не создан, создаём
mkfifo /tmp/screenshot_pipe 2>/dev/null

# слушаем и конвертируем
while IFS= read -r q; do
    [ -z "$q" ] && continue                  # защита от пустых строк
    png="${q%.qimg}.png"
    ./viewer "$q" "$png" && echo "→ $png"
done < /tmp/screenshot_pipe
