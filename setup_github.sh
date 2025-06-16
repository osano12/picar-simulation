#!/bin/bash

# Script pour configurer et pousser le projet sur GitHub
# Ce script vous guide à travers le processus de création d'un dépôt GitHub et de push du code

# Fonction pour afficher les messages d'information
info() {
    echo -e "\033[0;34m[INFO]\033[0m $1"
}

# Fonction pour afficher les messages d'erreur
error() {
    echo -e "\033[0;31m[ERROR]\033[0m $1"
}

# Fonction pour afficher les messages de succès
success() {
    echo -e "\033[0;32m[SUCCESS]\033[0m $1"
}

# Vérifier si git est installé
if ! command -v git &> /dev/null; then
    error "Git n'est pas installé. Veuillez l'installer avant de continuer."
    exit 1
fi

# Vérifier si le répertoire est déjà un dépôt git
if [ -d ".git" ]; then
    info "Ce répertoire est déjà un dépôt Git."
else
    info "Initialisation du dépôt Git..."
    git init
    success "Dépôt Git initialisé."
fi

# Demander le nom d'utilisateur GitHub
read -p "Entrez votre nom d'utilisateur GitHub: " github_username

# Demander le nom du dépôt
read -p "Entrez le nom du dépôt GitHub (par défaut: picar-simulator): " repo_name
repo_name=${repo_name:-picar-simulator}

# Demander une description du dépôt
read -p "Entrez une description pour le dépôt (par défaut: Simulateur interactif pour le PiCar X): " repo_description
repo_description=${repo_description:-"Simulateur interactif pour le PiCar X"}

# Ajouter tous les fichiers
info "Ajout des fichiers au dépôt..."
git add .

# Faire le premier commit
info "Création du premier commit..."
git commit -m "Initial commit: PiCar X Simulator"

# Configurer la branche principale
git branch -M main

# Ajouter le remote
info "Configuration du dépôt distant..."
git remote add origin "https://github.com/$github_username/$repo_name.git"

# Instructions pour créer le dépôt sur GitHub
echo ""
success "Configuration locale terminée!"
echo ""
info "Maintenant, vous devez créer un dépôt sur GitHub:"
echo ""
echo "1. Allez sur https://github.com/new"
echo "2. Entrez '$repo_name' comme nom de dépôt"
echo "3. Entrez '$repo_description' comme description"
echo "4. Choisissez si le dépôt doit être public ou privé"
echo "5. Ne cochez PAS 'Initialize this repository with a README'"
echo "6. Cliquez sur 'Create repository'"
echo ""
read -p "Appuyez sur Entrée une fois que vous avez créé le dépôt sur GitHub..."

# Pousser le code vers GitHub
info "Envoi du code vers GitHub..."
git push -u origin main

# Vérifier si le push a réussi
if [ $? -eq 0 ]; then
    success "Le code a été envoyé avec succès vers GitHub!"
    echo ""
    echo "Votre dépôt est maintenant disponible à l'adresse:"
    echo "https://github.com/$github_username/$repo_name"
    echo ""
    echo "Pour cloner ce dépôt sur un autre ordinateur, utilisez:"
    echo "git clone https://github.com/$github_username/$repo_name.git"
else
    error "Une erreur s'est produite lors de l'envoi du code vers GitHub."
    echo ""
    echo "Vérifiez que:"
    echo "1. Le dépôt existe sur GitHub"
    echo "2. Vous avez les droits d'accès nécessaires"
    echo "3. Vous êtes connecté à Internet"
    echo ""
    echo "Vous pouvez réessayer manuellement avec:"
    echo "git push -u origin main"
fi