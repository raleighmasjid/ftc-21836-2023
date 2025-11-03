package org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg;

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.ANY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.ANYCOLOR;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.colors;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * Implementation of Arshad Anas's CENTERSTAGE Optimal Placement Algorithm <br>
 * @author Arshad Anas
 */
public final class PlacementCalculator {
    public static final int[] initialColors = {7, 5, 5};

    private Backdrop backdrop;
    public final int[] colorsLeft = {5, 5, 5};

    private final ArrayList<Pixel>
            optimalPlacements = new ArrayList<>(),
            colorsToGetSPixels = new ArrayList<>();

    private ArrayList<Pixel> setLineSPixels;
    private boolean auton, specifyColors = false;

    public PlacementCalculator(boolean auton) {
        this.auton = auton;
    }
    public PlacementCalculator() {
        this(false);
    }

    /**
     * @return Whether the provided {@link Pixel} is part of a special case of mosaic case priorities
     * present in the perfect backdrop layout, where the up case is preferred over right or left to allow
     * the formation of another mosaic to the right
     */
    private boolean upMosaicIsHelpful(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        if (!(x == 3 && (y == 3 || y == 9))) return false;
        if (y == 9) return upMosaicIsHelpful(new Pixel(x, 3, pixel.color));

        Pixel[] shouldBeColored = {
                backdrop.get(5, y),
                backdrop.get(6, y),
                backdrop.get(6, y + 1),
        };
        for (Pixel p : shouldBeColored) if (!(p.color == EMPTY || p.color.isColored())) return false;

        Pixel[] shouldBeWhite = {
                backdrop.get(5, y - 1),
                backdrop.get(6, y - 1),
        };
        for (Pixel p : shouldBeWhite) if (!(p.color == EMPTY || p.color == Pixel.Color.WHITE)) return false;

        return isMosaicPossible();
    }

    /**
     * @return The three possible mosaic cases for the provided {@link Pixel}'s coordinates <br>
     * Case priorities are hardcoded to aid in finite-space packing
     */
    private Pixel[][] getPossibleMosaics(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        Pixel[] up = {
                backdrop.get(x, y),
                backdrop.get(x, y + 1),
                backdrop.get(x - 1 + 2 * (y % 2), y + 1)
        };
        Pixel[] right = {
                backdrop.get(x, y),
                backdrop.get(x + 1, y),
                backdrop.get(x + (y % 2), y + 1)
        };
        Pixel[] left = {
                backdrop.get(x, y),
                backdrop.get(x - 1, y),
                backdrop.get(x + (y % 2) - 1, y + 1)
        };
        if (x == 1) return new Pixel[][]{left, up, right};
        if (x == 6) return new Pixel[][]{right, up, left};
        if (upMosaicIsHelpful(pixel)) return new Pixel[][]{up, right, left};
        if (x == 5 || x == 3) return new Pixel[][]{right, left, up};
        return new Pixel[][]{left, right, up};
    }

    /**
     * Subtract values from {@link #colorsLeft} based on the colors present on {@link #backdrop}
     */
    private void countColorsLeft() {
        for (Pixel[] row : backdrop.slots) for (Pixel pixel : row) if (pixel.color.isColored()) {
            int index = pixel.color.ordinal();
            colorsLeft[index] = max(0, colorsLeft[index] - 1);
        }
    }

    /**
     * Iterate through {@link #backdrop}, detect and mark valid mosaics, mark invalid mosaics
     * by {@link Pixel}.mosaic attribute and add any {@link Pixel}s required to complete a mosaic
     * to {@link #optimalPlacements} and {@link #colorsToGetSPixels} <br>
     * After that, any required {@link Pixel} with ANYCOLOR as its color is given a specific color
     * as per the values of {@link #colorsLeft}
     */
    private void scanForMosaics() {
        for (Pixel[] row : backdrop.slots) for (Pixel pixel : row) pixel.mosaic = null;
        for (int y = 0; y < backdrop.slots.length; y++) for (int x : iterationXs(y)) {
            Pixel pixel = backdrop.get(x, y);

            if (pixel.mosaic != null || !pixel.color.isColored() || touchingAdjacentMosaic(pixel, false))
                continue;

            Pixel[][] possibleMosaics = getPossibleMosaics(pixel);
            Pixel[] pixels = {};

            boolean isMosaic = false;
            for (Pixel[] pMosaic : possibleMosaics) {
                boolean sameColor = pMosaic[0].color == pMosaic[1].color && pMosaic[1].color == pMosaic[2].color;
                boolean diffColors = pMosaic[0].color != pMosaic[1].color && pMosaic[1].color != pMosaic[2].color && pMosaic[0].color != pMosaic[2].color;
                boolean allColored = pMosaic[1].color.isColored() && pMosaic[2].color.isColored();
                isMosaic = sameColor || (diffColors && allColored);
                if (isMosaic) {
                    pixels = pMosaic;
                    break;
                }
            }

            if (isMosaic) {
                for (Pixel p : pixels) p.mosaic = pixel;
                for (Pixel p : pixels) {
                    if (touchingAdjacentMosaic(p, false)) {
                        invalidateMosaic(p.mosaic);
                        break;
                    }
                }
                if (pixels[0].mosaic.color != Pixel.Color.INVALID) backdrop.mosaicCount++;
                continue;
            }

            pMosaics:
            for (Pixel[] pMosaic : possibleMosaics) {
                for (Pixel a : pMosaic) if (a.color == Pixel.Color.WHITE) continue pMosaics;
                for (Pixel a : pMosaic)
                    if (a.color.isColored() || a.color == EMPTY) a.mosaic = pixel;

                if (
                        !touchingAdjacentMosaic(pMosaic[0], false) &&
                                !touchingAdjacentMosaic(pMosaic[1], false) &&
                                !touchingAdjacentMosaic(pMosaic[2], false)
                ) {
                    if (pMosaic[1].color.isColored() && pMosaic[2].color == EMPTY) {
                        oneRemainingCase(pixel, pMosaic[2], pMosaic[1]);
                    } else if (pMosaic[2].color.isColored() && pMosaic[1].color == EMPTY) {
                        oneRemainingCase(pixel, pMosaic[1], pMosaic[2]);
                    } else if (pMosaic[1].color == EMPTY && pMosaic[2].color == EMPTY) {

                        Pixel p1 = new Pixel(pMosaic[1], ANYCOLOR);
                        Pixel p2 = new Pixel(pMosaic[2], ANYCOLOR);

                        p1.scoreValue += 10 * .5;
                        p2.scoreValue += 10 * .5;

                        p1.mHelper = true;
                        p2.mHelper = true;

                        optimalPlacements.add(p1);
                        optimalPlacements.add(p2);

                        colorsToGetSPixels.add(new Pixel(p1, EMPTY));
                        colorsToGetSPixels.add(new Pixel(p2, EMPTY));

                        countExpendedPixels(pixel, p1, p2);
                    }
                    if (pMosaic[1].color == EMPTY || pMosaic[2].color == EMPTY) {
                        invalidateMosaic(pixel);
                        if (pMosaic[1].color == EMPTY && pMosaic[2].color == EMPTY) {
                            Pixel invalid = new Pixel(pixel, Pixel.Color.INVALID);
                            for (Pixel[] bMosaic : possibleMosaics)
                                for (Pixel a : bMosaic) a.mosaic = invalid;
                        }
                        break;
                    }
                }
                invalidateMosaic(pixel);

            }
        }
        removeDuplicates(colorsToGetSPixels);
        removeDuplicates(optimalPlacements);

        for (Pixel p : colorsToGetSPixels) optimalPlacements.addAll(getSupportPixels(p));
        removeDuplicates(optimalPlacements);
        removeUnsupportedPixels(optimalPlacements);
        removeOverridingPixels(optimalPlacements);
    }

    private void countExpendedPixels(Pixel p1, Pixel p2, Pixel p3) {
        int c1 = p1.color.ordinal();
        int c2 = (c1 + 1) % 3;
        int c3 = (c1 + 2) % 3;

        if (colorsLeft[c2] >= 1 && colorsLeft[c3] >= 1) {
            colorsLeft[c2]--;
            colorsLeft[c3]--;
            p2.recommended = colors[c2];
            p3.recommended = colors[c3];
        } else if (colorsLeft[c1] >= 2) {
            colorsLeft[c1] -= 2;
            p2.recommended = p3.recommended = colors[c1];
        }
    }

    private Pixel.Color getRemainingColor(Pixel.Color c1, Pixel.Color c2) {
        if (c1 == EMPTY || c2 == EMPTY) return ANYCOLOR;
        if (c1 == c2) return c1;
        ArrayList<Pixel.Color> colors = new ArrayList<>(Arrays.asList(Pixel.Color.GREEN, Pixel.Color.PURPLE, Pixel.Color.YELLOW));
        colors.remove(c1);
        colors.remove(c2);
        return colors.get(0);
    }

    private void oneRemainingCase(Pixel p1, Pixel fillLoc, Pixel p2) {
        Pixel.Color remainingColor = getRemainingColor(p1.color, p2.color);
        Pixel p3 = new Pixel(fillLoc, remainingColor);
        p3.recommended = remainingColor;
        p3.scoreValue += 10;
        p3.mHelper = true;
        optimalPlacements.add(p3);
        colorsToGetSPixels.add(new Pixel(p3, EMPTY));

        int index = p3.color.ordinal();
        colorsLeft[index] = max(0, colorsLeft[index] - 1);
    }

    private boolean touchingAdjacentColor(Pixel pixel) {
        for (Pixel neighbor : backdrop.getNeighbors(pixel)) if (neighbor.color.isColored()) {
                return true;
        }
        return false;
    }

    private boolean touchingAdjacentMosaic(Pixel pixel, boolean includeEmpties) {
        for (Pixel neighbor : backdrop.getNeighbors(pixel)) {
            if (neighbor.mosaic != pixel.mosaic) {

                if (neighbor.color.isColored()) return true;

                if (includeEmpties && neighbor.color == EMPTY && neighbor.mosaic != null) {

                    Pixel[][] pMosaics = getPossibleMosaics(backdrop.get(neighbor.mosaic));
                    ArrayList<Pixel[]> activePMosaics = new ArrayList<>();
                    pMosaics:
                    for (Pixel[] pMosaic : pMosaics) {
                        for (Pixel p : pMosaic) if (p.mosaic == null) continue pMosaics;
                        activePMosaics.add(pMosaic);
                    }

                    boolean[] pMosaicsTouchingPixel = new boolean[activePMosaics.size()];

                    for (int i = 0; i < activePMosaics.size(); i++) {
                        for (Pixel p : activePMosaics.get(i)) {
                            if (backdrop.touching(p, pixel)) {
                                pMosaicsTouchingPixel[i] = true;
                                break;
                            }
                        }
                    }

                    if (Backdrop.allTrue(pMosaicsTouchingPixel)) return true;

                }
            }
        }
        return false;
    }

    private void removeUnsupportedPixels(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (backdrop.isSupported(pixel)) pixels.add(pixel);
    }

    private void removeDuplicates(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (!pixel.isIn(pixels)) pixels.add(pixel);
    }

    private void removeOverridingPixels(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (backdrop.get(pixel).color == EMPTY) pixels.add(pixel);
    }

    private void invalidateMosaic(Pixel mosaic) {
        Pixel invMosaic = new Pixel(mosaic, Pixel.Color.INVALID);
        for (int y = 0; y < backdrop.slots.length; y++) for (int x : iterationXs(y)) {
            Pixel pixel = backdrop.slots[y][x];
            if (pixel.mosaic == mosaic && (pixel.color.isColored() || pixel.color == EMPTY)) {
                pixel.mosaic = invMosaic;
            }
        }

        for (int y = 0; y < backdrop.slots.length; y++) for (int x : iterationXs(y)) {
            Pixel pixel = backdrop.slots[y][x];
            if (pixel.color.isColored() && touchingAdjacentMosaic(pixel, false)) {
                pixel.mosaic = invMosaic;
            }
        }
    }

    private ArrayList<Pixel> getSupportPixels(Pixel pixel) {
        ArrayList<Pixel> sPixels = new ArrayList<>();
        if (pixel.color == EMPTY) {
            sPixels.add(getSafePixel(pixel));
            Pixel s1 = backdrop.get(pixel.x, pixel.y - 1);
            Pixel s2 = backdrop.get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1);
            sPixels.addAll(getSupportPixels(s1));
            sPixels.addAll(getSupportPixels(s2));
        }
        removeDuplicates(sPixels);
        removeOverridingPixels(sPixels);
        return sPixels;
    }

    private Pixel getSetLineGoal() {
        int highestY = backdrop.getHighestPixelY();
        int setY = highestY >= 5 ? 8 :
                highestY >= 2 ? 5 :
                        2;

        int leastSPixels = 100;
        Pixel bestSetPixel = backdrop.get(6, 8);

        setPixels:
        for (int x : iterationXs(setY)) {
            Pixel pixel = backdrop.get(x, setY);
            if (pixel.color == Pixel.Color.INVALID) continue;
            ArrayList<Pixel> sPixels = getSupportPixels(pixel);
            if (auton) for (Pixel p : sPixels) {
                if (p.isIn(colorsToGetSPixels)) continue setPixels;
            }
            if (sPixels.size() < leastSPixels) {
                leastSPixels = sPixels.size();
                bestSetPixel = pixel;
            }
        }

        return bestSetPixel;
    }

    private void scanForSetLinePixels() {
        Pixel setLineGoal = getSetLineGoal();
        setLineSPixels = getSupportPixels(setLineGoal);
        if (setLineGoal.y <= 8) optimalPlacements.addAll(setLineSPixels);
        removeUnsupportedPixels(optimalPlacements);
    }

    private void scanForEmptySpot() {
        for (int y = 0; y < backdrop.slots.length; y++) for (int x : iterationXs(y)) {
            Pixel pixel = backdrop.get(x, y);
            if (pixel.color == EMPTY && !pixel.isIn(optimalPlacements) && backdrop.isSupported(pixel)) {
                optimalPlacements.add(getSafePixel(pixel));
                return;
            }
        }
    }

    public static int[] iterationXs(int y) {
        return (y % 2 == 0) ? new int[]{1, 6, 2, 5, 3, 4} : new int[]{0, 6, 1, 5, 2, 4, 3};
    }

    private Pixel getSafePixel(Pixel pixel) {
        Pixel p1 = new Pixel(pixel,
                touchingAdjacentColor(pixel) || touchingAdjacentMosaic(pixel, true) || noSpaceForMosaics(pixel) || auton || !isMosaicPossible() ? Pixel.Color.WHITE :
                    specifyColors ? getFirstColor() :
                    Pixel.Color.ANY
        );
        if (p1.color == ANY) {
            p1.recommended = getFirstColor();
            p1.scoreValue += 10 / 3.0;
        }
        return p1;
    }

    private Pixel.Color getFirstColor() {
        int max = max(max(colorsLeft[0], colorsLeft[1]), colorsLeft[2]);
        int index = 0;
        for (int i = 0; i < colorsLeft.length; i++) if (colorsLeft[i] == max) index = i;
        return colors[index];
    }

    private boolean isMosaicPossible() {
        int p = colorsLeft[0], y = colorsLeft[1], g = colorsLeft[2];
        return p >= 3 || y >= 3 || g >= 3 || (p >= 1 && y >= 1 && g >= 1);
    }

    private boolean noSpaceForMosaics(Pixel pixel) {
        Pixel[][] pMosaics = getPossibleMosaics(pixel);
        boolean[] pMosaicsBlocked = new boolean[pMosaics.length];
        for (int i = 0; i < pMosaics.length; i++) {
            if (!(pMosaics[i][1].color == EMPTY) ||
                !(pMosaics[i][2].color == EMPTY) ||
                touchingAdjacentMosaic(pMosaics[i][1], true) ||
                touchingAdjacentMosaic(pMosaics[i][2], true)
            ) pMosaicsBlocked[i] = true;
        }
        return Backdrop.allTrue(pMosaicsBlocked);
    }

    private void sortPixelsToPlace() {
        for (Pixel pixel : optimalPlacements) {

//          if (pixel.color == WHITE) pixel.scoreValue += 11 / 9.0;
            
            for (Pixel mosaicPixel : colorsToGetSPixels) {
                if (pixel.equals(mosaicPixel)) continue;
                ArrayList<Pixel> mosaicSPixels = getSupportPixels(mosaicPixel);
                if (pixel.isIn(mosaicSPixels)) {
                    pixel.scoreValue += mosaicPixel.scoreValue / (double) mosaicSPixels.size();
                    pixel.mHelper = true;
                }
            }
//            if (pixel.color.matches(ANYCOLOR)) pixel.scoreValue += abs(3 - pixel.x) / 3.0;

            if (pixel.isIn(setLineSPixels)) pixel.scoreValue += 10 / (double) setLineSPixels.size();
        }
        Collections.sort(optimalPlacements);
    }

    public ArrayList<Pixel> getOptimalPlacements(Backdrop backdrop) {
        this.backdrop = backdrop;
        backdrop.mosaicCount = 0;

        colorsLeft[0] = initialColors[0];
        colorsLeft[1] = initialColors[1];
        colorsLeft[2] = initialColors[2];

        optimalPlacements.clear();
        colorsToGetSPixels.clear();

        countColorsLeft();
        scanForMosaics();
        scanForSetLinePixels();
        scanForEmptySpot();

        removeDuplicates(optimalPlacements);
        removeOverridingPixels(optimalPlacements);
        removeUnsupportedPixels(optimalPlacements);

        sortPixelsToPlace();

//        System.out.println(colorsLeft[0]);
//        System.out.println(colorsLeft[1]);
//        System.out.println(colorsLeft[2]);

        return new ArrayList<>(optimalPlacements);
    }
}
