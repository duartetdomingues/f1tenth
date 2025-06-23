#!/usr/bin/env python3
# Script to record ROS2 bags with topic selection using fzf

import subprocess
import shlex

def get_topics():
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True, check=True)
        topics = result.stdout.strip().split('\n')
        return topics
    except subprocess.CalledProcessError:
        print("Erro ao obter lista de tópicos.")
        return []

def select_topics(topics):
    try:
        # Envia os tópicos para o fzf com seleção múltipla (-m)
        fzf = subprocess.run(['fzf', '-m'], input='\n'.join(topics), text=True, stdout=subprocess.PIPE)
        selected = fzf.stdout.strip().split('\n')
        return selected
    except Exception as e:
        print(f"Erro ao selecionar tópicos: {e}")
        return []

def record_bag(selected_topics):
    if not selected_topics:
        print("Nenhum tópico selecionado.")
        return

    cmd = ['ros2', 'bag', 'record'] + selected_topics
    print(f"\n📦 A gravar rosbag com os tópicos:\n{selected_topics}\n")
    print("💡 Usa Ctrl+C para parar a gravação.\n")

    subprocess.run(cmd)

if __name__ == "__main__":
    topics = get_topics()
    if not topics:
        exit(1)
    
    print("🔍 A mostrar tópicos no fzf... seleciona com <tab> e confirma com <enter>\n")
    selected = select_topics(topics)
    record_bag(selected)
