#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>
 
/* ============================================================
 *  STRUCTURE DE DONNÉES
 * ============================================================ */
 
typedef struct {
    int   id;   /* Identifiant unique du drone          */
    float x;    /* Coordonnée X (longitude)             */
    float y;    /* Coordonnée Y (latitude)              */
    float z;    /* Coordonnée Z (altitude)              */
} Drone;
 
/* ============================================================
 *  RÉSULTAT : paire de drones les plus proches
 * ============================================================ */
 
typedef struct {
    int   id1;      /* ID du premier drone  */
    int   id2;      /* ID du second drone   */
    float distance; /* Distance euclidienne */
} PaireProche;
 
/* ============================================================
 *  FONCTION : distance euclidienne 3D entre deux drones
 *  Paramètres : pointeurs vers deux drones
 *  Retour     : distance (float)
 * ============================================================ */
float distance_euclidienne(Drone *a, Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}
 
/* ============================================================
 *  FONCTION : comparateur pour tri par axe X (qsort)
 *  Utilisé pour trier le tableau de pointeurs par coordonnée X
 * ============================================================ */
int comparer_par_x(const void *a, const void *b) {
    Drone *da = *(Drone **)a;
    Drone *db = *(Drone **)b;
    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}
 
/* ============================================================
 *  FONCTION : comparateur pour tri par axe Y (qsort)
 * ============================================================ */
int comparer_par_y(const void *a, const void *b) {
    Drone *da = *(Drone **)a;
    Drone *db = *(Drone **)b;
    if (da->y < db->y) return -1;
    if (da->y > db->y) return  1;
    return 0;
}
 
/* ============================================================
 *  FONCTION : force brute pour petits sous-ensembles (≤ 3)
 *  Paramètres : tableau de pointeurs, taille, résultat
 * ============================================================ */
void force_brute(Drone **pts, int n, PaireProche *res) {
    int i, j;
    for (i = 0; i < n - 1; i++) {
        for (j = i + 1; j < n; j++) {
            /* Navigation par arithmétique de pointeurs */
            Drone *a = *(pts + i);
            Drone *b = *(pts + j);
            float d = distance_euclidienne(a, b);
            if (d < res->distance) {
                res->distance = d;
                res->id1 = a->id;
                res->id2 = b->id;
            }
        }
    }
}
 
/* ============================================================
 *  FONCTION : vérification dans la bande centrale (strip)
 *  La bande contient les drones dont |x - milieu_x| < delta
 * ============================================================ */
void verifier_bande(Drone **bande, int taille_bande, float delta, PaireProche *res) {
    int i, j;
 
    /* Tri de la bande par Y pour optimiser les comparaisons */
    qsort(bande, taille_bande, sizeof(Drone *), comparer_par_y);
 
    for (i = 0; i < taille_bande; i++) {
        Drone *a = *(bande + i);
 
        for (j = i + 1; j < taille_bande; j++) {
            Drone *b = *(bande + j);
 
            /* On sort de la bande si la distance Y dépasse delta */
            if ((b->y - a->y) >= delta) break;
 
            float d = distance_euclidienne(a, b);
            if (d < res->distance) {
                res->distance = d;
                res->id1 = a->id;
                res->id2 = b->id;
            }
        }
    }
}
 
/* ============================================================
 *  FONCTION PRINCIPALE : Closest Pair - Divide & Conquer
 *  Paramètres : tableau trié par X, taille
 *  Retour     : paire la plus proche
 * ============================================================ */
PaireProche closest_pair(Drone **pts_x, int n) {
    PaireProche res;
    res.distance = FLT_MAX;
    res.id1 = -1;
    res.id2 = -1;
 
    /* Cas de base : force brute pour ≤ 3 drones */
    if (n <= 3) {
        force_brute(pts_x, n, &res);
        return res;
    }
 
    /* ---- DIVIDE : couper en deux moitiés ---- */
    int milieu = n / 2;
 
    /* Point médian (via arithmétique de pointeurs) */
    Drone *drone_milieu = *(pts_x + milieu);
 
    /* Récursion sur la moitié gauche */
    PaireProche gauche = closest_pair(pts_x, milieu);
 
    /* Récursion sur la moitié droite */
    PaireProche droite = closest_pair(pts_x + milieu, n - milieu);
 
    /* ---- CONQUER : choisir le meilleur des deux ---- */
    res = (gauche.distance < droite.distance) ? gauche : droite;
    float delta = res.distance;
 
    /* ---- MERGE : vérifier la bande centrale ---- */
    /* Construire la bande : drones dont |x - milieu_x| < delta */
    Drone **bande = (Drone **)malloc(n * sizeof(Drone *));
    if (!bande) {
        fprintf(stderr, "Erreur : allocation mémoire bande échouée\n");
        exit(EXIT_FAILURE);
    }
 
    int taille_bande = 0;
    int i;
    for (i = 0; i < n; i++) {
        Drone *d = *(pts_x + i);
        if (fabsf(d->x - drone_milieu->x) < delta) {
            *(bande + taille_bande) = d;
            taille_bande++;
        }
    }
 
    /* Vérifier les paires dans la bande */
    verifier_bande(bande, taille_bande, delta, &res);
 
    free(bande);
    return res;
}
 
/* ============================================================
 *  FONCTION : initialiser l'essaim avec des coordonnées aléatoires
 * ============================================================ */
void initialiser_essaim(Drone *essaim, int n) {
    int i;
    for (i = 0; i < n; i++) {
        Drone *d = essaim + i;   /* Arithmétique de pointeurs */
        d->id = i;
        d->x  = ((float)rand() / RAND_MAX) * 10000.0f;
        d->y  = ((float)rand() / RAND_MAX) * 10000.0f;
        d->z  = ((float)rand() / RAND_MAX) * 1000.0f;
    }
}
 
/* ============================================================
 *  PROGRAMME PRINCIPAL
 * ============================================================ */
int main(void) {
 
    const int N = 10000; /* Nombre de drones */
 
    printf("==============================================\n");
    printf("  SYSTÈME DE DÉTECTION DE COLLISION - UAV\n");
    printf("  Essaim de %d drones\n", N);
    printf("==============================================\n\n");
 
    /* ---- Allocation dynamique de l'essaim ---- */
    Drone *essaim = (Drone *)malloc(N * sizeof(Drone));
    if (!essaim) {
        fprintf(stderr, "Erreur : allocation mémoire essaim échouée\n");
        return EXIT_FAILURE;
    }
 
    /* ---- Initialisation avec coordonnées aléatoires ---- */
    srand((unsigned int)time(NULL));
    initialiser_essaim(essaim, N);
 
    printf("[1] Essaim initialisé : %d drones alloués en mémoire\n", N);
 
    /* ---- Construction du tableau de pointeurs (pour tri) ---- */
    Drone **pts = (Drone **)malloc(N * sizeof(Drone *));
    if (!pts) {
        fprintf(stderr, "Erreur : allocation tableau de pointeurs échouée\n");
        free(essaim);
        return EXIT_FAILURE;
    }
 
    int i;
    for (i = 0; i < N; i++) {
        *(pts + i) = essaim + i;  /* Arithmétique de pointeurs */
    }
 
    /* ---- Tri initial par X ---- */
    qsort(pts, N, sizeof(Drone *), comparer_par_x);
    printf("[2] Tri initial par axe X effectué\n");
 
    /* ---- Lancement de l'algorithme ---- */
    printf("[3] Algorithme Closest Pair (Divide & Conquer) en cours...\n\n");
 
    clock_t debut = clock();
    PaireProche resultat = closest_pair(pts, N);
    clock_t fin = clock();
 
    double temps = (double)(fin - debut) / CLOCKS_PER_SEC * 1000.0;
 
    /* ---- Affichage du résultat ---- */
    printf("==============================================\n");
    printf("  RÉSULTAT : PAIRE LA PLUS PROCHE DÉTECTÉE\n");
    printf("==============================================\n");
    printf("  Drone #%d  <-->  Drone #%d\n", resultat.id1, resultat.id2);
    printf("  Distance minimale : %.6f mètres\n", resultat.distance);
    printf("  Temps d'exécution : %.3f ms\n", temps);
 
    /* Affichage des coordonnées des deux drones */
    Drone *d1 = essaim + resultat.id1;
    Drone *d2 = essaim + resultat.id2;
    printf("\n  Drone #%d : (x=%.2f, y=%.2f, z=%.2f)\n",
           d1->id, d1->x, d1->y, d1->z);
    printf("  Drone #%d : (x=%.2f, y=%.2f, z=%.2f)\n",
           d2->id, d2->x, d2->y, d2->z);
    printf("==============================================\n");
 
    /* ---- Libération de la mémoire ---- */
    free(pts);
    free(essaim);
 
    printf("\n[4] Mémoire libérée. Programme terminé.\n");
 
    return EXIT_SUCCESS;
}
 
