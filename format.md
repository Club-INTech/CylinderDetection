# Format des packets de communication avec le HL
Tout se base sur une communication à base de texte, comme le reste de l'architecture du HL actuel (2020).

Un packet suit la forme suivante:

| Orientation de la caméra                      | Nombre de formes (N)          | Formes                                        |
|-----------------------------------------------|-------------------------------|-----------------------------------------------|
| X Y Z, 3 float à 5 chiffres après la virgule  | Entier allant de 0 à INT_MAX  | N formes, voir la section Définition de Forme |

## Définition de forme
Une forme est décrite avec la structure suivante:

| Type de forme                         | Hauteur de la forme (cm)   | Position locale de la forme                      | Position sur la table estimée                          |
|---------------------------------------|----------------------------|--------------------------------------------------|--------------------------------------------------------|
| 'cylinder' seulement pour le moment   | Float                      | X Y Z, 3 float, dans le référentiel de la caméra | X Y Z, 3 float, dans le référentiel estimé de la table |
