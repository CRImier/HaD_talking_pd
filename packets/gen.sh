npx bit-field --bits 16 -i header.json > header.svg; #gwenview header.svg
npx bit-field --bits 32 --lanes 4 -i pdo.json > pdo.svg; #gwenview pdo.svg
npx bit-field --bits 32 --lanes 4 -i request.json > request.svg; #gwenview request.svg
gwenview request.svg
