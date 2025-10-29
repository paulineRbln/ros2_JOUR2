### ROBLIN Pauline / César / Mathis 
## 29/10/25

# Ex1 
- Package contenant mon modèle urdf est contenu dans le dossier mon_modele_urdf 
- Version corrigée du prof directement placée dans le workspace pour pouvoir travailler dessus sans conflit avec le mien

# Ex2 
- Moveit : je n'ai pas réussi à le lancer 
- Setup Assistant permet de définir un ensemble de configurations : collisions, poses, controllers
- On enregistre la configuration : generate package 
- On peut ensuite contrôler le robot en jouant sur les groupes, execution de mouvement, retour à une pose
- 2ddl = très limité en terme de positions et mouvements possibles

# FRANKA 
- clone sur git
- installation des dépendances
- build

# Ex4 
- Nous avons réussi à planifier et exécuter le mouvement du bras vers une position précise

# Ex6 
- Création des box
- Ajout à la liste des objets à éviter
- planification de la trajectoire
- Contrôle de l'évitement des objets

# Ex7 & 8 (dans exercice 08)
- Création des 2 box + cyclindre
- Ajout à une liste commune
- plannification de la trajectoire avec les différentes étapes
  -> Tout fonctionne sauf au moment d'attraper le cyclindre (mauvaise calibration de la position)
