export function movedPieces(model) {
  const loose = [];
  const inCells = [];
  const inFlowers = [];
  for (const piece of model.pieces) {
    if (!piece.loose && piece.cell === undefined && piece.flower === undefined) {
      continue;
    }
    (piece.cell !== undefined ? inCells : piece.flower !== undefined ? inFlowers : loose).push(piece);
  }
  return loose.concat(inCells).concat(inFlowers);
}

export function asTheBenchServesIt(model, robotIn) {
  return Object.assign({}, model, { moved: movedPieces(model), robotIn });
}
