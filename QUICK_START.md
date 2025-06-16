# 🚀 Guide de Démarrage Rapide - PiCar X Simulator

## 📋 Options de Lancement

### Option 1: Mode Démo (Recommandé pour commencer)
**Le plus simple et le plus rapide**

```bash
cd /home/osano/Documents/PICARX
./launch_demo.sh
```

✅ **Avantages:**
- Démarrage immédiat (< 1 minute)
- Aucune installation complexe requise
- Interface web fonctionnelle
- Données de capteurs simulées
- Exécution de code Python

### Option 2: Version Complète avec ROS 2
**Pour une simulation complète avec Gazebo**

```bash
cd /home/osano/Documents/PICARX
./launch_full.sh
```

✅ **Avantages:**
- Simulation physique complète
- ROS 2 + Gazebo
- Rendu 3D réaliste
- Tous les capteurs simulés

## 🌐 Accès à l'Interface Web

Une fois lancé, ouvrez votre navigateur et allez à:
- **Local**: http://localhost:8080
- **Réseau**: http://[votre-ip]:8080

## 🎮 Fonctionnalités Disponibles

### Mode Démo
- ✅ Interface web interactive
- ✅ Visualisation des données de capteurs
- ✅ Exécution de code Python
- ✅ Flux vidéo simulé
- ✅ Scénarios d'apprentissage

### Version Complète
- ✅ Tout du mode démo +
- ✅ Simulation physique Gazebo
- ✅ ROS 2 Humble
- ✅ Capteurs réalistes
- ✅ Environnements 3D

## 🛠️ Dépannage

### Problème: "Permission denied"
```bash
chmod +x launch_demo.sh
chmod +x launch_full.sh
```

### Problème: Docker non disponible
```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl start docker
sudo usermod -aG docker $USER
# Redémarrez votre session
```

### Problème: Port 8080 occupé
```bash
# Trouver le processus utilisant le port
sudo lsof -i :8080
# Arrêter le processus
sudo kill -9 [PID]
```

## 📚 Premiers Pas

1. **Lancez le mode démo** avec `./launch_demo.sh`
2. **Ouvrez votre navigateur** sur http://localhost:8080
3. **Explorez l'interface** - observez les données des capteurs
4. **Testez l'exécution de code** avec le scénario "Mode Démo"
5. **Regardez le flux vidéo** simulé en temps réel

## 🎯 Scénarios Disponibles

- **Mode Démo**: Introduction aux fonctionnalités
- **Tutoriel 1**: Premiers pas avec le PiCar X
- **Plus de scénarios** disponibles en mode complet

## 📞 Support

Si vous rencontrez des problèmes:
1. Vérifiez que vous êtes dans le bon répertoire
2. Assurez-vous que Python 3 est installé
3. Pour la version complète, vérifiez que Docker fonctionne
4. Consultez les logs d'erreur dans le terminal

---

**Conseil**: Commencez toujours par le mode démo pour vous familiariser avec l'interface avant de passer à la version complète.