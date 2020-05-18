# Format des packets de communication avec le HL
Tout se base sur une communication à base de texte, comme le reste de l'architecture du HL actuel (2020).
Le channel utilisé est '!§'.

Un packet suit la forme suivante:

| Orientation de la caméra                      | Nombre de formes (N)          | Formes                                        |
|-----------------------------------------------|-------------------------------|-----------------------------------------------|
| X Y Z, 3 float à 5 chiffres après la virgule  | Entier allant de 0 à INT_MAX  | N formes, voir la section Définition de Forme |

## Définition de forme
Une forme est décrite avec la structure suivante:

| Type de forme                         | Couleur                          | Hauteur de la forme (m)   | Position locale de la forme                      | Position sur la table estimée                          |
|---------------------------------------|----------------------------------|----------------------------|--------------------------------------------------|--------------------------------------------------------|
| 'cylinder' seulement pour le moment   | 'green' ou 'red' pour le moment  | Float                      | X Y Z, 3 float, dans le référentiel de la caméra | X Y Z, 3 float, dans le référentiel estimé de la table |

## Regex utilisable pour tout décoder:
https://regex101.com/r/kAOAIz/1

`^(?<angleX>([0-9]+(\.[0-9]*)?)) (?<angleY>([0-9]+(\.[0-9]*)?)) (?<angleZ>([0-9]+(\.[0-9]*)?)) (?<shapeCount>[0-9]+) (?<shape>(?<type>[a-z]+ (?<color>[a-z]+) (?<height>([0-9]+(\.[0-9]*)?)) (?<localX>([0-9]+(\.[0-9]*)?)) (?<localY>([0-9]+(\.[0-9]*)?)) (?<localZ>([0-9]+(\.[0-9]*)?)) (?<globalX>([0-9]+(\.[0-9]*)?)) (?<globalY>([0-9]+(\.[0-9]*)?)) (?<globalZ>([0-9]+(\.[0-9]*)?))) ?)+`