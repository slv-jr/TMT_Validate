#!/bin/bash
# StormWings — installation sur Raspberry Pi 5 (Raspberry Pi OS Bookworm 64-bit)
# Compatible aussi avec Pi 4B (chemin /boot/firmware/config.txt vs /boot/config.txt auto-détecté)
#
# Ce script doit être lancé avec sudo :
#     sudo bash scripts/install.sh
#
# Il prépare le Pi pour la navigation autonome :
#  1. Désactive le Bluetooth (qui occupe l'UART /dev/serial0 par défaut)
#  2. Active l'UART hardware sur GPIO 14/15
#  3. Désactive le shell série (libère le port pour MAVLink)
#  4. Installe les dépendances Python
#  5. Installe le service systemd (optionnel, demande confirmation)
#  6. Crée les dossiers logs/

set -euo pipefail

INSTALL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
echo "════════════════════════════════════════════════"
echo "StormWings — installation depuis $INSTALL_DIR"
echo "════════════════════════════════════════════════"

if [ "$(id -u)" -ne 0 ]; then
    echo "❌ Ce script doit être lancé avec sudo"
    exit 1
fi

# ─── 1. UART hardware ───
echo
echo "[1/6] Configuration UART hardware…"
CONFIG_TXT=/boot/firmware/config.txt
[ ! -f "$CONFIG_TXT" ] && CONFIG_TXT=/boot/config.txt

# Désactiver Bluetooth
if ! grep -q "^dtoverlay=disable-bt" "$CONFIG_TXT"; then
    echo "dtoverlay=disable-bt" >> "$CONFIG_TXT"
    echo "  ✓ Bluetooth désactivé dans $CONFIG_TXT"
else
    echo "  ✓ Bluetooth déjà désactivé"
fi

# Activer UART
if ! grep -q "^enable_uart=1" "$CONFIG_TXT"; then
    echo "enable_uart=1" >> "$CONFIG_TXT"
    echo "  ✓ UART activé"
fi

# Désactiver le service hciuart
systemctl disable hciuart 2>/dev/null || true
systemctl stop hciuart 2>/dev/null || true

# Désactiver le shell série
CMDLINE_TXT=/boot/firmware/cmdline.txt
[ ! -f "$CMDLINE_TXT" ] && CMDLINE_TXT=/boot/cmdline.txt
if grep -q "console=serial0" "$CMDLINE_TXT"; then
    sed -i 's/console=serial0,[0-9]* //g' "$CMDLINE_TXT"
    echo "  ✓ Console série retirée de cmdline.txt"
fi
systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true

# ─── 2. Permissions ports ───
echo
echo "[2/6] Permissions ports série…"
USERNAME="${SUDO_USER:-pi}"
usermod -aG dialout "$USERNAME"
echo "  ✓ Utilisateur $USERNAME ajouté au groupe dialout"

# ─── 3. Dépendances système ───
echo
echo "[3/6] Mise à jour APT et dépendances système…"
apt-get update
apt-get install -y --no-install-recommends \
    python3 python3-pip python3-venv \
    git build-essential \
    libatlas-base-dev \
    python3-numpy python3-matplotlib

# ─── 4. Dépendances Python ───
echo
echo "[4/6] Installation dépendances Python…"
cd "$INSTALL_DIR"
sudo -u "$USERNAME" pip3 install --break-system-packages --upgrade -r requirements.txt
echo "  ✓ Dépendances Python installées"

# ─── 5. Dossier logs ───
echo
echo "[5/6] Création dossiers de logs…"
mkdir -p "$INSTALL_DIR/logs"
chown -R "$USERNAME:$USERNAME" "$INSTALL_DIR/logs"
echo "  ✓ $INSTALL_DIR/logs prêt"

# ─── 6. Service systemd (optionnel) ───
echo
echo "[6/6] Service systemd…"
read -p "  Installer le service systemd pour démarrage auto ? (o/N) " -n 1 -r ANSWER
echo
if [[ "$ANSWER" =~ ^[OoYy]$ ]]; then
    if [ -f "$INSTALL_DIR/scripts/stormwings.service" ]; then
        # Adapter les chemins
        sed -e "s|__INSTALL_DIR__|$INSTALL_DIR|g" \
            -e "s|__USER__|$USERNAME|g" \
            "$INSTALL_DIR/scripts/stormwings.service" \
            > /etc/systemd/system/stormwings.service
        systemctl daemon-reload
        systemctl enable stormwings.service
        echo "  ✓ Service installé et activé"
        echo "    Commandes utiles :"
        echo "      sudo systemctl start stormwings"
        echo "      sudo systemctl status stormwings"
        echo "      sudo journalctl -u stormwings -f"
    else
        echo "  ⚠ scripts/stormwings.service introuvable, étape ignorée"
    fi
else
    echo "  → Service non installé (lancement manuel : DRONE_ID=U1B1 python3 main.py)"
fi

echo
echo "════════════════════════════════════════════════"
echo "✅ Installation terminée !"
echo "════════════════════════════════════════════════"
echo
echo "⚠️  REDÉMARRAGE OBLIGATOIRE pour que les modifs UART/Bluetooth"
echo "   prennent effet :"
echo "       sudo reboot"
echo
echo "Après le reboot :"
echo "  1. Vérifier que /dev/serial0 existe :  ls -l /dev/serial0"
echo "  2. Brancher le Cube Orange+ et l'ESP32 LoRa"
echo "  3. Lancer le test de connexion :"
echo "       cd $INSTALL_DIR"
echo "       DRONE_ID=U1B1 python3 -m tests.test_connexion"
echo
