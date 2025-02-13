#!/bin/bash

# Script pour :
# 1. Copier grp(premierBateau) vers grp(premierBateau) sur les bateaux suivants
# 2. Copier calib_(bateau).txt vers grp(premierBateau)/calib_(bateau).txt sur le premier bateau
# 3. Exécuter consensus.py sur chaque bateau en se plaçant dans grp(premierBateau)

########################################
# Lecture du fichier config.txt
########################################
CONFIG_FILE="config.txt"

# Vérifie que le fichier config.txt existe
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Le fichier $CONFIG_FILE est introuvable!"
  exit 1
fi

# Lit toutes les lignes dans un tableau
declare -a BATEAUX
while IFS= read -r line || [ -n "$line" ]; do
  # On ignore les lignes vides ou commentaires si nécessaire
  if [ -n "$line" ]; then
    BATEAUX+=("$line")
  fi
done < "$CONFIG_FILE"

# Vérifie qu'on a au moins 2 bateaux
if [ ${#BATEAUX[@]} -lt 2 ]; then
  echo "Il faut au moins deux bateaux dans $CONFIG_FILE."
  exit 1
fi

# Le premier bateau
PREMIER="${BATEAUX[0]}"

########################################
# 1. Copie de grp(premier) en série 
#    de BATEAUX[i] vers BATEAUX[i+1]
########################################

echo "========== ETAPE 1 : COPIES EN CHAINE DU DOSSIER grp${PREMIER} =========="
for (( i=0; i<${#BATEAUX[@]}-1; i++ ))
do
    SRC="${BATEAUX[$i]}"
    DST="${BATEAUX[$i+1]}"
    
    echo "Copie de grp${PREMIER} du bateau ${SRC} vers le bateau ${DST} ..."
    
    # Commande scp pour copier le dossier grp(PREMIER)
    # -r pour copie récursive
    scp -r "ue32@172.20.25.2${SRC}:~/grp${PREMIER}" "ue32@172.20.25.2${DST}:~/grp${PREMIER}"
    
    echo "=> Copie terminée de ${SRC} vers ${DST}."
    echo
done

echo "========== ETAPE 2 : COPIE DES FICHIERS calib_(bateau).txt DE grp${BATEAU} VERS grp${PREMIER} (SUR CHAQUE BATEAU) =========="
for BATEAU in "${BATEAUX[@]}"; do
  echo "Sur le bateau ${BATEAU}, copie de ~/grp${BATEAU}/calib_${BATEAU}.txt vers ~/grp${PREMIER}/calib_${BATEAU}.txt ..."
  
  ssh "ue32@172.20.25.2${BATEAU}" \
      "cp ~/grp${BATEAU}/calib_${BATEAU}.txt ~/grp${PREMIER}/calib_${BATEAU}.txt"

  if [ $? -eq 0 ]; then
    echo "=> Fichier calib_${BATEAU}.txt copié avec succès dans ~/grp${PREMIER}/ sur le bateau ${BATEAU}."
  else
    echo "=> ERREUR : Impossible de copier calib_${BATEAU}.txt sur le bateau ${BATEAU}."
  fi
  echo
done

########################################
# 3. Exécution de consensus.py 
#    en parallèle sur tous les bateaux
########################################

echo "========== ETAPE 3 : EXECUTION DE consensus.py SUR TOUS LES BATEAUX (EN PARALLÈLE) =========="
for BATEAU in "${BATEAUX[@]}"; do
  echo "Lancement sur le bateau ${BATEAU}..."

  # Méthode 1 : ssh -f + nohup 
  # -f : ssh se met en tâche de fond immédiatement
  # nohup + & : le script Python continue même si la session SSH se termine
  ssh -f "ue32@172.20.25.2${BATEAU}" \
      "cd ~/grp${PREMIER} && nohup python3.5 consensus.py > consensus_${BATEAU}.log 2>&1 &"

  echo "=> Commande lancée sur le bateau ${BATEAU}, pas d'attente locale."
  echo
done

echo "========== TOUTES LES COMMANDES ONT ÉTÉ LANCÉES EN PARALLÈLE =========="

