setwd("C:\\Users\\Jens\\Google Drive\\JensWoehrleHCI\\Evalution")
getwd()

data <- read.csv("data_new_csv.csv", header=TRUE)
#data$restposition_version <- paste(data$restposition, "-", data$version)
#data$score <- data$goodnessMapping + data$easeOfUse

head(data)
data$Geschlecht                             #gets specific Column


table(data$restposition)                            #sums up values of a specific column

nrow(data)                                 # Count the rows in A

summary(data)                    #Summary of a specific column
# is accessed as A$year

#column Operations
#A$newcol <- A$my1 + A$my2               # Makes a new column in A
#newvar <- A$my1 - A$my2                 # Makes a new R object "newvar"
#A$my1 <- NULL                           # Removes the column "my1"

# You might find these useful, to "look around" a dataset --
str(data)
summary(data)
library(Hmisc)          # This requires that you've installed the Hmisc packag
contents(data)
describe(data)


####################EVERYTHING TOGETHER DIFFERENT ORDER ######################################

########OVERALL BOXPLOTS Goodness, EaseofUse, Score
competence_mean <- aggregate(Competence~Version, data, mean)
competence_var <- aggregate(Competence~Version, data, var)
sensory_mean <- aggregate(Sensory.and.Imaginative.Immersion~Version, data, mean)
sensory_var <- aggregate(Sensory.and.Imaginative.Immersion~Version, data, var)
flow_mean <- aggregate(Flow~Version, data, mean)
flow_var <- aggregate(Flow~Version, data, var)
tension_mean <- aggregate(Tension~Version, data, mean)
tension_var <- aggregate(Tension~Version, data, var)
challenge_mean <- aggregate(Challenge~Version, data, mean)
challenge_var <- aggregate(Challenge~Version, data, var)
negative_mean <- aggregate(Negative.Effect~Version, data, mean)
negative_var <- aggregate(Negative.Effect~Version, data, var)
positive_mean <- aggregate(Positiv.Effect~Version, data, mean)
positive_var <- aggregate(Positiv.Effect~Version, data, var)
score_mean <- aggregate(Score~Version, data, mean)
score_var <- aggregate(Score~Version, data, var)

op <- "Competence"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper(op),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="darkgreen")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(competence_mean$Competence, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(competence_var$Competence, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Sensory.and.Imaginative.Immersion"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper("Sensory and Imaginative Immersion"),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="pink")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(sensory_mean$Sensory.and.Imaginative.Immersion, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(sensory_var$Sensory.and.Imaginative.Immersion, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Flow"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper(op),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="yellow")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(flow_mean$Flow, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(flow_var$Flow, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Tension"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper(op),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="red")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(tension_mean$Tension, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(tension_var$Tension, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Challenge"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper(op),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="blue")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(challenge_mean$Challenge, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(challenge_var$Challenge, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Negative.Effect"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper("Negative Effect"),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="green")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(negative_mean$Negative.Effect, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(negative_var$Negative.Effect, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Positiv.Effect"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper("Positive Effect"),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="purple")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(positive_mean$Positiv.Effect, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(positive_var$Positiv.Effect, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)

op <- "Score"
boxplot(eval(parse(text=op))~Version, 
        data=data, 
        main=toupper(op),
        font.main=3, cex.main=1.2, 
        xlab="Version", 
        ylab="1-5 Rating", font.lab=3, 
        col="orange")
#text(1:length(unique(data$restposition_version)), 0, labels = round(my_mean$goodnessMapping, digits = 2), pos=1, cex=0.6)
#text(1:length(unique(data$restposition_version)), -1, labels = round(my_var$goodnessMapping, digits = 2), pos=1, cex=0.6)
mtext("Mean", side = 3, line = 0.7, at = 0.3, cex=0.6)
mtext("Var", side = 3, line = 0, at = 0.3, cex=0.6)
mtext(round(score_mean$Score, digits = 2), side = 3, line = 0.7, at = 1:length(unique(data$Version)), cex=0.6)
mtext(round(score_var$Score, digits = 2), side = 3, line = 0, at = 1:length(unique(data$Version)), cex=0.6)



#################BarPlots Diagramm ######
df<-data


groupedData <- aggregate(data$Score,
                         by = list(rpos = data$Fachrichtung, ver = data$Version),
                         FUN = function(x) c(mean = mean(x), sd = sd(x),
                                             n = length(x)))

groupedData <- do.call(data.frame, groupedData)

groupedData$se <- groupedData$x.sd / sqrt(groupedData$x.n)

colnames(groupedData) <- c("rpos", "ver", "mean", "sd", "n", "se")

groupedData$names <- c(paste(groupedData$rpos, "-",
                             groupedData$ver))

par(mar = c(5, 6, 4, 5) + 0.1)

plotTop <- max(groupedData$mean) +
  groupedData[groupedData$mean == max(groupedData$mean), 6] * 3



tapply(groupedData$mean, list(groupedData$rpos, groupedData$ver),
       function(x) c(x = x))
tabbedMeans <- tapply(groupedData$mean, list(groupedData$rpos,
                                        groupedData$ver),
                      function(x) c(x = x))
tabbedSE <- tapply(groupedData$se, list(groupedData$rpos,
                                   groupedData$ver),
                   function(x) c(x = x))

barCenters <- barplot(height = tabbedMeans,
                      beside = TRUE, las = 1,
                      ylim = c(0, plotTop),
                      cex.names = 0.75,
                      main = "Score",
                      ylab = "1-5 Rating",
                      xlab = "Version",
                      border = "black", axes = TRUE,
                      legend.text = TRUE,
                      args.legend = list(title = "Restposition", 
                                         x = "topright",
                                         cex = .7))

segments(barCenters, tabbedMeans - tabbedSE * 2, barCenters,
         tabbedMeans + tabbedSE * 2, lwd = 1.5)

arrows(barCenters, tabbedMeans - tabbedSE * 2, barCenters, 
       tabbedMeans + tabbedSE * 2, lwd = 1.5, angle = 90, code = 3, length = 0.05)





library(ggplot2)

dodge <- position_dodge(width = 0.9)
limits <- aes(ymax = groupedData$mean + groupedData$se,
              ymin = groupedData$mean - groupedData$se)

p <- ggplot(data = groupedData, aes(x = factor(rpos), y = mean,
                               fill = factor(ver)))

p + geom_bar(stat = "identity",
             position = position_dodge(0.9)) +
  geom_errorbar(limits, position = position_dodge(0.9),
                width = 0.25) +
  labs(x = "Background", y = "1-5 Rating") +
  ggtitle("Score") +
  scale_fill_discrete(name = "Version") +
  geom_text(aes(label=n), position=position_dodge(width=0.9), vjust=10.25)

#yes only, und dann auf easo of use oder Goodnessmapping machen ;-)